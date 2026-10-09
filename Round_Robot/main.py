#!/usr/bin/env python3
from pathlib import Path
import os, subprocess, sys
p = Path(__file__).resolve().parent
env = os.environ.copy()
env['GZ_SIM_RESOURCE_PATH'] = str(p / 'models') + (os.pathsep + env['GZ_SIM_RESOURCE_PATH'] if env.get('GZ_SIM_RESOURCE_PATH') else '')
sys.exit(subprocess.call(['gz', 'sim', '-r', '--render-engine', 'ogre', str(p / 'world.sdf')], env=env))
