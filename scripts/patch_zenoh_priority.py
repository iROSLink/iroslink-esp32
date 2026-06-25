Import("env")
import os

target = os.path.join(
    env.subst("$PROJECT_LIBDEPS_DIR"),
    env.subst("$PIOENV"),
    "zenoh-pico", "src", "system", "arduino", "esp32", "system.c"
)

OLD = "    .priority = configMAX_PRIORITIES / 2,"
NEW = "    .priority = Z_TASK_PRIORITY,"

if os.path.isfile(target):
    text = open(target).read()
    if OLD in text:
        open(target, "w").write(text.replace(OLD, NEW))
        print("patch_zenoh_priority: applied")
    else:
        print("patch_zenoh_priority: already applied")
else:
    print(f"patch_zenoh_priority: target not found: {target}")
