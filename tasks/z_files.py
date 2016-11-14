""" File Management """

import os
import re


def purge(pattern):
    """ remove files """
    for f in os.listdir("."):
        if re.search(pattern, f):
            os.remove(os.path.join(".", f))


def rename(pattern, new_string):
    """ rename files """
    i = 0

    for f in os.listdir("."):
        if re.search(pattern, f):
            os.rename(f, f.replace(pattern, new_string))
            i += 1
    print(str(i) + " files renamed")


rename("Arm_cube_table_4096s9a__LE_TOSL__SOFT__T3", "3d_arm_table_4096s_9a")
