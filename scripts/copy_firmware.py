import shutil
import os
Import("env")

def copy_firmware(source, target, env):
    src = str(target[0])
    dest = os.path.join(env.subst("$PROJECT_DIR"), f"firmware_{env['PIOENV']}.bin")
    shutil.copy(src, dest)
    print(f"\n>>> Firmware copiato in: firmware_{env['PIOENV']}.bin\n")

env.AddPostAction("$BUILD_DIR/firmware.bin", copy_firmware)
