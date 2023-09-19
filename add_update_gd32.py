Import("env")
env.AddCustomTarget(
    "Update gd32",
    None,
    "pio pkg update -g -p gd32"
)