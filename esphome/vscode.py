from __future__ import print_function

import json
import os

from esphome.config import load_config, _format_vol_invalid
from esphome.core import CORE
from esphome.py_compat import text_type, safe_input
from esphome.yaml_util import ESPHomeDataBase


def _get_invalid_range(res, invalid):
    # type: (Config, vol.Invalid) -> Optional[DocumentRange]
    obj = res.get_deepest_value_for_path(invalid.path)
    if isinstance(obj, ESPHomeDataBase) and obj.esp_range is not None:
        return obj.esp_range
    return None


def _dump_range(range):
    # type: (Optional[DocumentRange]) -> Optional[dict]
    if range is None:
        return None
    return {
        'document': range.start_mark.document,
        'start_line': range.start_mark.line,
        'start_col': range.start_mark.column,
        'end_line': range.end_mark.line,
        'end_col': range.end_mark.column,
    }


class VSCodeResult(object):
    def __init__(self):
        self.yaml_errors = []
        self.validation_errors = []

    def dump(self):
        return json.dumps({
            'type': 'result',
            'yaml_errors': self.yaml_errors,
            'validation_errors': self.validation_errors,
        })

    def add_yaml_error(self, message):
        self.yaml_errors.append({
            'message': message,
        })

    def add_validation_error(self, range_, message):
        self.validation_errors.append({
            'range': _dump_range(range_),
            'message': message,
        })


def read_config(args):
    while True:
        CORE.reset()
        data = json.loads(safe_input())
        assert data['type'] == 'validate'
        CORE.vscode = True
        CORE.ace = args.ace
        f = data['file']
        if CORE.ace:
            CORE.config_path = os.path.join(args.configuration, f)
        else:
            CORE.config_path = data['file']
        vs = VSCodeResult()
        try:
            res = load_config()
        except Exception as err:  # pylint: disable=broad-except
            vs.add_yaml_error(text_type(err))
        else:
            for err in res.errors:
                try:
                    range_ = _get_invalid_range(res, err)
                    vs.add_validation_error(range_, _format_vol_invalid(err, res))
                except:
                    continue
        print(vs.dump())
