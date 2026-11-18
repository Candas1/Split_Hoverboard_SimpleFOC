Import("env")

filters = [{"library":"Simple FOC", "option":"motor","values":["BLDC","Stepper"],"enable":False},
           {"library":"Simple FOC", "option":"driver","values":["2PWM","3PWM","4PWM","6PWM"],"enable":False},
           {"library":"Simple FOC", "option":"current","values":["None","GenericCurrentSense","InlineCurrentSense","LowsideCurrentSense"],"enable":False},
           {"library":"Simple FOC", "option":"platform","values":["stm32","gd32","esp32","esp8266","rp2040","teensy","atmega","samd","due","nrf52","portenta","renesas"],"enable":False},
           {"library":"Simple FOC", "option":"sensor","values":["None","Encoder","MagneticSensorSPI","MagneticSensorI2C","MagneticSensorAnalog","MagneticSensorPWM","HallSensor"],"enable":False}]

def skip_from_build(node):
    """
    `node.name` - a name of File System Node
    `node.get_path()` - a relative path
    `node.get_abspath()` - an absolute path
     to ignore file from a build process, just return None
    """
    
    for filter in filters:
        # Library name is in the path
        if filter["library"] in node.get_path():
            # there was a valid option
            if filter["enable"]:
                # for each value in the list except the selected option (removed)
                for value in filter["values"]:
                    # if value is file name
                    if value in node.name:
                        # Return None for exclude
                        return None
    return node

# Read value from other environments
config = env.GetProjectConfig()
for filter in filters:
    if config.has_section(filter["library"]):
        if config.has_option(filter["library"],"custom_" + filter["option"]):
            value = config.get(filter["library"],"custom_" + filter["option"])
            if value in filter["values"]:
                filter["values"].remove(value)
                filter["enable"] = True

# Register callback
env.AddBuildMiddleware(skip_from_build, "*")