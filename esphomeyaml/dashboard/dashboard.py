# pylint: disable=wrong-import-position
from __future__ import print_function

import collections
import hmac
import json
import logging
import multiprocessing
import os
import random
import subprocess
import threading

from tornado.log import access_log
from typing import Optional

from esphomeyaml import const
from esphomeyaml.__main__ import get_serial_ports
from esphomeyaml.core import EsphomeyamlError
from esphomeyaml.helpers import run_system_command
from esphomeyaml.storage_json import StorageJSON, ext_storage_path
from esphomeyaml.util import shlex_quote

try:
    import tornado
    import tornado.gen
    import tornado.ioloop
    import tornado.iostream
    import tornado.process
    import tornado.web
    import tornado.websocket
    import tornado.concurrent
except ImportError as err:
    tornado = None

_LOGGER = logging.getLogger(__name__)
CONFIG_DIR = ''
PASSWORD = ''


# pylint: disable=abstract-method
class BaseHandler(tornado.web.RequestHandler):
    def is_authenticated(self):
        return not PASSWORD or self.get_secure_cookie('authenticated') == 'yes'


# pylint: disable=abstract-method, arguments-differ
class EsphomeyamlCommandWebSocket(tornado.websocket.WebSocketHandler):
    def __init__(self, application, request, **kwargs):
        super(EsphomeyamlCommandWebSocket, self).__init__(application, request, **kwargs)
        self.proc = None
        self.closed = False

    def on_message(self, message):
        if PASSWORD and self.get_secure_cookie('authenticated') != 'yes':
            return
        if self.proc is not None:
            return
        command = self.build_command(message)
        _LOGGER.info(u"Running command '%s'", ' '.join(shlex_quote(x) for x in command))
        self.proc = tornado.process.Subprocess(command,
                                               stdout=tornado.process.Subprocess.STREAM,
                                               stderr=subprocess.STDOUT)
        self.proc.set_exit_callback(self.proc_on_exit)
        tornado.ioloop.IOLoop.current().spawn_callback(self.redirect_stream)

    @tornado.gen.coroutine
    def redirect_stream(self):
        while True:
            try:
                data = yield self.proc.stdout.read_until_regex('[\n\r]')
            except tornado.iostream.StreamClosedError:
                break
            if data.endswith('\r') and random.randrange(100) < 90:
                continue
            try:
                data = data.replace('\033', '\\033')
            except UnicodeDecodeError:
                data = data.encode('ascii', 'backslashreplace')
            self.write_message({'event': 'line', 'data': data})

    def proc_on_exit(self, returncode):
        if not self.closed:
            _LOGGER.debug("Process exited with return code %s", returncode)
            self.write_message({'event': 'exit', 'code': returncode})

    def on_close(self):
        self.closed = True
        if self.proc is not None and self.proc.returncode is None:
            _LOGGER.debug("Terminating process")
            self.proc.proc.terminate()

    def build_command(self, message):
        raise NotImplementedError


class EsphomeyamlLogsHandler(EsphomeyamlCommandWebSocket):
    def build_command(self, message):
        js = json.loads(message)
        config_file = CONFIG_DIR + '/' + js['configuration']
        return ["esphomeyaml", config_file, "logs", '--serial-port', js["port"], '--escape']


class EsphomeyamlRunHandler(EsphomeyamlCommandWebSocket):
    def build_command(self, message):
        js = json.loads(message)
        config_file = os.path.join(CONFIG_DIR, js['configuration'])
        return ["esphomeyaml", config_file, "run", '--upload-port', js["port"],
                '--escape', '--use-esptoolpy']


class EsphomeyamlCompileHandler(EsphomeyamlCommandWebSocket):
    def build_command(self, message):
        js = json.loads(message)
        config_file = os.path.join(CONFIG_DIR, js['configuration'])
        return ["esphomeyaml", config_file, "compile"]


class EsphomeyamlValidateHandler(EsphomeyamlCommandWebSocket):
    def build_command(self, message):
        js = json.loads(message)
        config_file = os.path.join(CONFIG_DIR, js['configuration'])
        return ["esphomeyaml", config_file, "config"]


class EsphomeyamlCleanMqttHandler(EsphomeyamlCommandWebSocket):
    def build_command(self, message):
        js = json.loads(message)
        config_file = os.path.join(CONFIG_DIR, js['configuration'])
        return ["esphomeyaml", config_file, "clean-mqtt"]


class EsphomeyamlCleanHandler(EsphomeyamlCommandWebSocket):
    def build_command(self, message):
        js = json.loads(message)
        config_file = os.path.join(CONFIG_DIR, js['configuration'])
        return ["esphomeyaml", config_file, "clean"]


class EsphomeyamlHassConfigHandler(EsphomeyamlCommandWebSocket):
    def build_command(self, message):
        js = json.loads(message)
        config_file = os.path.join(CONFIG_DIR, js['configuration'])
        return ["esphomeyaml", config_file, "hass-config"]


class SerialPortRequestHandler(BaseHandler):
    def get(self):
        if not self.is_authenticated():
            self.redirect('/login')
            return
        ports = get_serial_ports()
        data = []
        for port, desc in ports:
            if port == '/dev/ttyAMA0':
                desc = 'UART pins on GPIO header'
            split_desc = desc.split(' - ')
            if len(split_desc) == 2 and split_desc[0] == split_desc[1]:
                # Some serial ports repeat their values
                desc = split_desc[0]
            data.append({'port': port, 'desc': desc})
        data.append({'port': 'OTA', 'desc': 'Over-The-Air'})
        self.write(json.dumps(sorted(data, reverse=True)))


class WizardRequestHandler(BaseHandler):
    def post(self):
        from esphomeyaml import wizard

        if not self.is_authenticated():
            self.redirect('/login')
            return
        kwargs = {k: ''.join(v) for k, v in self.request.arguments.iteritems()}
        destination = os.path.join(CONFIG_DIR, kwargs['name'] + '.yaml')
        wizard.wizard_write(path=destination, **kwargs)
        self.redirect('/?begin=True')


class DownloadBinaryRequestHandler(BaseHandler):
    def get(self):
        if not self.is_authenticated():
            self.redirect('/login')
            return

        configuration = self.get_argument('configuration')
        storage_path = ext_storage_path(CONFIG_DIR, configuration)
        storage_json = StorageJSON.load(storage_path)
        if storage_json is None:
            self.send_error()
            return

        path = storage_json.firmware_bin_path
        self.set_header('Content-Type', 'application/octet-stream')
        filename = '{}.bin'.format(storage_json.name)
        self.set_header("Content-Disposition", 'attachment; filename="{}"'.format(filename))
        with open(path, 'rb') as f:
            while 1:
                data = f.read(16384)  # or some other nice-sized chunk
                if not data:
                    break
                self.write(data)
        self.finish()


def _list_yaml_files():
    files = []
    for file in os.listdir(CONFIG_DIR):
        if not file.endswith('.yaml'):
            continue
        if file.startswith('.'):
            continue
        if file == 'secrets.yaml':
            continue
        files.append(file)
    files.sort()
    return files


def _list_dashboard_entries():
    files = _list_yaml_files()
    return [DashboardEntry(file) for file in files]


class DashboardEntry(object):
    def __init__(self, filename):
        self.filename = filename
        self._storage = None
        self._loaded_storage = False

    @property
    def full_path(self):  # type: () -> str
        return os.path.join(CONFIG_DIR, self.filename)

    @property
    def storage(self):  # type: () -> Optional[StorageJSON]
        if not self._loaded_storage:
            self._storage = StorageJSON.load(ext_storage_path(CONFIG_DIR, self.filename))
            self._loaded_storage = True
        return self._storage

    @property
    def address(self):
        if self.storage is None:
            return None
        return self.storage.address

    @property
    def name(self):
        if self.storage is None:
            return self.filename[:-len('.yaml')]
        return self.storage.name

    @property
    def esp_platform(self):
        if self.storage is None:
            return None
        return self.storage.esp_platform

    @property
    def board(self):
        if self.storage is None:
            return None
        return self.storage.board


class MainRequestHandler(BaseHandler):
    def get(self):
        if not self.is_authenticated():
            self.redirect('/login')
            return

        begin = bool(self.get_argument('begin', False))
        entries = _list_dashboard_entries()
        version = const.__version__
        docs_link = 'https://beta.esphomelib.com/esphomeyaml/' if 'b' in version else \
            'https://esphomelib.com/esphomeyaml/'

        self.render("templates/index.html", entries=entries,
                    version=version, begin=begin, docs_link=docs_link)


def _ping_func(filename, address):
    if os.name == 'nt':
        command = ['ping', '-n', '1', address]
    else:
        command = ['ping', '-c', '1', address]
    rc, _, _ = run_system_command(*command)
    return filename, rc == 0


class PingThread(threading.Thread):
    def run(self):
        pool = multiprocessing.Pool(processes=8)

        while not STOP_EVENT.is_set():
            # Only do pings if somebody has the dashboard open
            PING_REQUEST.wait()
            PING_REQUEST.clear()

            def callback(ret):
                PING_RESULT[ret[0]] = ret[1]

            entries = _list_dashboard_entries()
            queue = collections.deque()
            for entry in entries:
                if entry.address is None:
                    PING_RESULT[entry.filename] = None
                    continue

                result = pool.apply_async(_ping_func, (entry.filename, entry.address),
                                          callback=callback)
                queue.append(result)

            while queue:
                item = queue[0]
                if item.ready():
                    queue.popleft()
                    continue

                try:
                    item.get(0.1)
                except multiprocessing.TimeoutError:
                    pass

                if STOP_EVENT.is_set():
                    pool.terminate()
                    return


class PingRequestHandler(BaseHandler):
    def get(self):
        if not self.is_authenticated():
            self.redirect('/login')
            return

        PING_REQUEST.set()
        self.write(json.dumps(PING_RESULT))


PING_RESULT = {}  # type: dict
STOP_EVENT = threading.Event()
PING_REQUEST = threading.Event()


class LoginHandler(BaseHandler):
    def get(self):
        self.write('<html><body><form action="/login" method="post">'
                   'Password: <input type="password" name="password">'
                   '<input type="submit" value="Sign in">'
                   '</form></body></html>')

    def post(self):
        password = str(self.get_argument("password", ''))
        password = hmac.new(password).digest()
        if hmac.compare_digest(PASSWORD, password):
            self.set_secure_cookie("authenticated", "yes")
        self.redirect("/")


def make_app(debug=False):
    def log_function(handler):
        if handler.get_status() < 400:
            log_method = access_log.info

            if isinstance(handler, SerialPortRequestHandler) and not debug:
                return
            if isinstance(handler, PingRequestHandler) and not debug:
                return
        elif handler.get_status() < 500:
            log_method = access_log.warning
        else:
            log_method = access_log.error

        request_time = 1000.0 * handler.request.request_time()
        log_method("%d %s %.2fms", handler.get_status(),
                   handler._request_summary(), request_time)

    static_path = os.path.join(os.path.dirname(__file__), 'static')
    app = tornado.web.Application([
        (r"/", MainRequestHandler),
        (r"/login", LoginHandler),
        (r"/logs", EsphomeyamlLogsHandler),
        (r"/run", EsphomeyamlRunHandler),
        (r"/compile", EsphomeyamlCompileHandler),
        (r"/validate", EsphomeyamlValidateHandler),
        (r"/clean-mqtt", EsphomeyamlCleanMqttHandler),
        (r"/clean", EsphomeyamlCleanHandler),
        (r"/hass-config", EsphomeyamlHassConfigHandler),
        (r"/download.bin", DownloadBinaryRequestHandler),
        (r"/serial-ports", SerialPortRequestHandler),
        (r"/ping", PingRequestHandler),
        (r"/wizard.html", WizardRequestHandler),
        (r'/static/(.*)', tornado.web.StaticFileHandler, {'path': static_path}),
    ], debug=debug, cookie_secret=PASSWORD, log_function=log_function)
    return app


def start_web_server(args):
    global CONFIG_DIR
    global PASSWORD

    if tornado is None:
        raise EsphomeyamlError("Attempted to load dashboard, but tornado is not installed! "
                               "Please run \"pip2 install tornado esptool\" in your terminal.")

    CONFIG_DIR = args.configuration
    if not os.path.exists(CONFIG_DIR):
        os.makedirs(CONFIG_DIR)

    # HassIO options storage
    PASSWORD = args.password
    if os.path.isfile('/data/options.json'):
        with open('/data/options.json') as f:
            js = json.load(f)
            PASSWORD = js.get('password') or PASSWORD

    if PASSWORD:
        PASSWORD = hmac.new(str(PASSWORD)).digest()
        # Use the digest of the password as our cookie secret. This makes sure the cookie
        # isn't too short. It, of course, enables local hash brute forcing (because the cookie
        # secret can be brute forced without making requests). But the hashing algorithm used
        # by tornado is apparently strong enough to make brute forcing even a short string pretty
        # hard.

    _LOGGER.info("Starting dashboard web server on port %s and configuration dir %s...",
                 args.port, CONFIG_DIR)
    app = make_app(args.verbose)
    app.listen(args.port)

    if args.open_ui:
        import webbrowser

        webbrowser.open('localhost:{}'.format(args.port))

    ping_thread = PingThread()
    ping_thread.start()
    try:
        tornado.ioloop.IOLoop.current().start()
    except KeyboardInterrupt:
        _LOGGER.info("Shutting down...")
        STOP_EVENT.set()
        PING_REQUEST.set()
        ping_thread.join()
