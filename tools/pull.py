#!/usr/bin/env python3

import glob
import os
import re
import subprocess

from termcolor import cprint

import b
from dockerise import run_on_docker


@run_on_docker
def register(command):
    command.help = "Pull data files from a robot"

    # Configuration
    command.add_argument("target", help="the target host to install the packages to")


@run_on_docker
def run(target, **kwargs):

    # sources = b.cmake_cache["NUCLEAR_MODULE_DATA_FILE_SOURCES"]
    # # If there is only a single file then the b script returns this as a string rather than a list
    # sources = sources if isinstance(sources, list) else [sources]
    # # Get list of config files
    # sources = [os.path.relpath(c, build_dr) for c in sources]

    # targets = b.cmake_cache["NUCLEAR_MODULE_DATA_FILE_TARGETS"]

    # Replace hostname with its IP address if the hostname is already known
    num_robots = 4
    target = {
        "{}{}".format(k, num): "159.65.25.93"
        for num in range(1, num_robots + 1)
        for k, v in zip(("nugus", "n", "i", "igus"), [num] * num_robots)
    }.get(target, target)

    print("target: ", target)

    # If no user, use our user
    user = "abi"

    # If no user, use our user
    if user is None:
        import getpass

        user = getpass.getuser()

    # Target location to install to
    target_dir = "{0}@{1}:/srv/users/{0}/nubots/".format(user, target)

    print("target_dir: ", target_dir)

    # Array of sources (local)
    sources = [
        "/home/nubots/NUbots/my-scripts/A.txt",
        "/home/nubots/NUbots/my-scripts/B.txt",
        "/home/nubots/NUbots/my-scripts/C.txt",
    ]

    # Array of targets (robot)
    targets = [
        "/srv/users/abi/nubots/scripts-2/A.txt",
        "/srv/users/abi/nubots/scripts-2/B.txt",
        "/srv/users/abi/nubots/scripts-2/C.txt",
    ]

    for s, t in zip(sources, targets):
        command = ["rsync", "-avzPL", "--checksum", "-e sshpass -p 'nubots2020' ssh"] + ["abi@159.65.25.93:" + t] + [s]
        print("command: ", command)
        subprocess.call(command)
