import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@cptskippy"]

DEPENDENCIES = ["uart"]

MULTI_CONF = True

ld2415h_ns = cg.esphome_ns.namespace("ld2415h")
LD2415HComponent = ld2415h_ns.class_("LD2415HComponent", cg.Component, uart.UARTDevice)

CONF_LD2415H_ID = "ld2415h_id"
CONF_PROTOCOL_MODE = "protocol_mode"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LD2415HComponent),
            cv.Optional(CONF_PROTOCOL_MODE, default="auto"): cv.enum(
                {"auto": "auto", "ascii": "ascii", "binary": "binary"}, lower=True
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2415h_uart",
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    
    if CONF_PROTOCOL_MODE in config:
        if config[CONF_PROTOCOL_MODE] == "binary":
            cg.add(var.set_binary_protocol(True))
        elif config[CONF_PROTOCOL_MODE] == "ascii":
            cg.add(var.set_binary_protocol(False))
        # "auto" mode will detect automatically
