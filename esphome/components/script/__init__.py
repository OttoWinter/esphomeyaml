from esphome import automation
from esphome.automation import ACTION_REGISTRY, maybe_simple_id
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID

script_ns = cg.esphome_ns.namespace('script')
Script = script_ns.class_('Script', cg.Trigger.template())
ScriptExecuteAction = script_ns.class_('ScriptExecuteAction', cg.Action)
ScriptStopAction = script_ns.class_('ScriptStopAction', cg.Action)

CONFIG_SCHEMA = automation.validate_automation({
    cv.Required(CONF_ID): cv.declare_id(Script),
})


def to_code(config):
    for conf in config:
        trigger = cg.new_Pvariable(conf[CONF_ID])
        yield automation.build_automation(trigger, [], conf)


@automation.register_action('script.execute', ScriptExecuteAction, maybe_simple_id({
    cv.Required(CONF_ID): cv.use_id(Script),
}))
def script_execute_action_to_code(config, action_id, template_arg, args):
    paren = yield cg.get_variable(config[CONF_ID])
    yield cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action('script.stop', ScriptStopAction, maybe_simple_id({
    cv.Required(CONF_ID): cv.use_id(Script)
}))
def script_stop_action_to_code(config, action_id, template_arg, args):
    paren = yield cg.get_variable(config[CONF_ID])
    yield cg.new_Pvariable(action_id, template_arg, paren)
