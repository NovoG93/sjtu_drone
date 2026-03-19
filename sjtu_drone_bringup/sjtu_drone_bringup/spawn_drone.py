#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Spawn drone in gz-sim using ros_gz_sim create."""

import subprocess
import sys
import tempfile


def main():
    if len(sys.argv) < 3:
        print('Usage: spawn_drone <urdf_xml> <model_name>')
        sys.exit(1)

    xml = sys.argv[1]
    name = sys.argv[2]

    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(xml)
        tmpfile = f.name

    subprocess.run(
        ['ros2', 'run', 'ros_gz_sim', 'create', '-file', tmpfile, '-name', name],
        check=True,
    )


if __name__ == '__main__':
    main()
