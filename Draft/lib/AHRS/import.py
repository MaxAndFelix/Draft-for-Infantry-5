import os
Import("env")
global_env = DefaultEnvironment()


env.Append(LINKFLAGS=['-Wl,--start-group'])
env.Append(LINKFLAGS=[os.path.abspath('source/AHRS.lib')])
