Import("env")
env.AddCustomTarget(
    "Update simpleFOC",
    None,
    "pio pkg update -l \"Simple FOC\""
)