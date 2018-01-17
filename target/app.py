import _app
import cstruct
queuemove = _app.queuemove

stepper_config = cstruct.CStruct(_app._stepper_config_structdef, _app._stepper_config_data)
