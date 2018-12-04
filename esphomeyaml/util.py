from __future__ import print_function

import io
import logging
import re
import sys

_LOGGER = logging.getLogger(__name__)


class Registry(dict):
    def register(self, name):
        def decorator(fun):
            self[name] = fun
            return fun

        return decorator


class ServiceRegistry(dict):
    def register(self, name, validator):
        def decorator(fun):
            self[name] = (validator, fun)
            return fun

        return decorator


def safe_print(message=""):
    try:
        print(message)
        return
    except UnicodeEncodeError:
        pass

    try:
        print(message.encode('ascii', 'backslashreplace'))
    except UnicodeEncodeError:
        print("Cannot print line because of invalid locale!")


def shlex_quote(s):
    if not s:
        return u"''"
    if re.search(r'[^\w@%+=:,./-]', s) is None:
        return s

    return u"'" + s.replace(u"'", u"'\"'\"'") + u"'"


class RedirectText(object):
    def __init__(self, out):
        self._out = out

    def __getattr__(self, item):
        return getattr(self._out, item)

    # pylint: disable=no-self-use
    def isatty(self):
        print("isatty: True")
        return True


def run_external_command(func, *cmd, **kwargs):
    def mock_exit(return_code):
        raise SystemExit(return_code)

    orig_argv = sys.argv
    orig_exit = sys.exit  # mock sys.exit
    full_cmd = u' '.join(shlex_quote(x) for x in cmd)
    _LOGGER.info(u"Running:  %s", full_cmd)

    sys.stdin = RedirectText(sys.stdin)
    sys.stdout = RedirectText(sys.stdout)
    sys.stderr = RedirectText(sys.stderr)

    capture_stdout = kwargs.get('capture_stdout', False)
    if capture_stdout:
        sys.stdout = io.BytesIO()

    try:
        sys.argv = list(cmd)
        sys.exit = mock_exit
        return func() or 0
    except KeyboardInterrupt:
        return 1
    except SystemExit as err:
        return err.args[0]
    except Exception as err:  # pylint: disable=broad-except
        _LOGGER.error(u"Running command failed: %s", err)
        _LOGGER.error(u"Please try running %s locally.", full_cmd)
    finally:
        sys.argv = orig_argv
        sys.exit = orig_exit

        if isinstance(sys.stdin, RedirectText):
            sys.stdin = sys.__stdin__
        if isinstance(sys.stdout, RedirectText):
            sys.stdout = sys.__stdout__
        if isinstance(sys.stderr, RedirectText):
            sys.stderr = sys.__stderr__

        if capture_stdout:
            # pylint: disable=lost-exception
            stdout = sys.stdout.getvalue()
            sys.stdout = sys.__stdout__
            return stdout
