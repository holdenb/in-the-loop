#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json

from waflib import Logs, Task
from waflib.Build import BuildContext
from waflib.Configure import ConfigurationContext
from waflib.Options import OptionsContext
from waflib.Task import Task as WTask


# ---------------------------------------------------------------------#
def options(opt: OptionsContext) -> None:
    """
    Define custom command-line options here if needed.
    """
    # 1) Load standard compiler flags into the Waf command-line system
    opt.load("compiler_cxx")
    opt.load("pkgconfig")

    # 2) Define your own option
    # opt.add_option('--use-clang', action='store_true', default=False,
    #                help='Force clang++ as the C++ compiler')
    # Can use options like so from within configure():
    # if conf.options.use_clang:
    #   pass


# ---------------------------------------------------------------------#
def configure(conf: ConfigurationContext) -> None:
    """
    Configure checks (e.g., find compilers, libraries).
    """
    # Force use of clang++
    conf.find_program("clang++", var="CXX")

    # 1) Detect the default compiler
    conf.load("compiler_cxx")
    conf.load("pkgconfig")

    # This will add -I/usr/include/eigen3 (and no libs, since it's header-only).
    conf.check_cfg(
        package="eigen3",
        uselib_store="EIGEN3",
        args="--cflags --libs",
        mandatory=True,
    )

    # 2) Now override with clang++
    conf.env.CXX = "clang++"
    # Make sure the 'command' field becomes clang++
    conf.env.CXX_NAME = "clang++"
    conf.env.CXXFLAGS += ["-std=c++20", "-Wall"]


# ---------------------------------------------------------------------#
def build(bld: BuildContext) -> None:
    """
    Build tasks for the internal library and the main program.
    """

    util_includes_abs = bld.path.find_node("include/util").abspath()
    util_sources = bld.path.ant_glob("src/util/*.cpp")
    bld.stlib(
        source=util_sources,
        target="util",
        includes=[util_includes_abs],
        use=["EIGEN3"],
    )

    itl_includes_abs = bld.path.find_node("include/itl").abspath()
    mpc_sources = bld.path.ant_glob("src/itl/*.cpp")
    bld.stlib(
        source=mpc_sources,
        target="itl",
        includes=[itl_includes_abs, util_includes_abs],
        use=["util", "EIGEN3"],
    )

    includes_abs = bld.path.find_node("include").abspath()
    sim_sources = bld.path.ant_glob("src/sim/*.cpp")
    bld.program(
        source=sim_sources,
        target="sim",
        includes=[includes_abs, util_includes_abs],
        use=["util", "itl", "EIGEN3"],
    )

    # Optionally gather more .cpp from deeper subdirectories, for example:
    # bld.path.ant_glob('src/mpc/**/*.cpp')
    # bld.path.ant_glob('src/itl/**/*.cpp')

    # all_cpp = bld.path.ant_glob('src/mpc/**/*.cpp')
    # Filter out anything named *_test.cpp
    # mpc_sources = [f for f in all_cpp if not f.name.endswith('_test.cpp')]

    # Attach a custom function to run after build
    bld.add_post_fun(generate_compilation_db)

    # Additional logging to verify the proj root, build dir, and top-level
    # src dir (waf project / wscript location).
    Logs.info(f"bld.path    = {bld.path.abspath()}")
    Logs.info(f"bld.bldnode = {bld.bldnode.abspath()}")
    Logs.info(f"bld.srcnode = {bld.srcnode.abspath()}")


# ---------------------------------------------------------------------#
# How This Works
#
# 1.) bld.program or bld.stlib creates:
#
# - One link task (cxxprogram, cxxstlib, etc.), which has a member list .compiled_tasks.
# - Several compile tasks (cxx), one per source file, that actually produce the .o files.
#
# 2.) We iterate over all tasks in the build groups. If we see:
#
# - A pure compile task (cxx or c): we record its commands.
# - A link task with .compiled_tasks: we iterate through those compiled tasks to get the real commands.
# - If it’s something else (e.g. a custom packaging task, or something that doesn’t compile code), we skip it.
# record_commands_for_task does the actual generation logic. This keeps your code DRY and handles both C and C++ tasks.
def is_compile_task(t: WTask) -> bool:
    """Return True if 't' is a c or cxx compile task."""
    c_class = Task.classes.get("c")
    cxx_class = Task.classes.get("cxx")

    return (c_class and isinstance(t, c_class)) or (
        cxx_class and isinstance(t, cxx_class)
    )


def generate_compilation_db(bld: BuildContext) -> None:
    """
    Gathers compilation commands from C/C++ tasks
    (including those referenced by link tasks) and writes compile_commands.json.
    """
    commands = []

    for group in bld.groups:
        for task in group:
            # 1) If this is a pure compile task (cxx or c),
            #    handle it directly:
            if is_compile_task(task):
                record_commands_for_task(task, bld, commands)

            # 2) If this is a link task (like cxxprogram/cxxstlib/cxxshlib),
            #    it may reference one or more compile tasks in task.compiled_tasks:
            elif hasattr(task, "compiled_tasks"):
                for ctask in task.compiled_tasks:
                    if is_compile_task(ctask):
                        record_commands_for_task(ctask, bld, commands)

    # This will write to the build/ directory
    # out_file = bld.path.find_or_declare('compile_commands.json').abspath()

    # Write compile_commands.json
    out_file = bld.bldnode.make_node("compile_commands.json").abspath()
    with open(out_file, "w") as f:
        json.dump(commands, f, indent=2)

    Logs.info(f"Wrote compile_commands.json to {out_file}")


def record_commands_for_task(task: WTask, bld: BuildContext, commands: list) -> None:
    """
    Given a single compile task (cxx or c), extract the command line for each source.
    """
    # Determine if this is C or C++
    if isinstance(task, Task.classes.get("cxx")):
        compiler = task.env.CXX_NAME or (task.env.CXX and task.env.CXX[0]) or "c++"
        flags = task.env.CXXFLAGS
    else:
        compiler = task.env.CC_NAME or (task.env.CC and task.env.CC[0]) or "cc"
        flags = task.env.CFLAGS

    command = [compiler] + flags

    incs = [f"-I{inc}" for inc in task.env.INCLUDES]
    defs = [f"-D{d}" for d in task.env.DEFINES]
    command += incs + defs

    # For each source file, create an entry
    for idx, src in enumerate(task.inputs):
        output = task.outputs[idx]
        cmd_list = command + ["-c", src.abspath(), "-o", output.abspath()]

        commands.append(
            {
                "directory": bld.bldnode.abspath(),
                "command": " ".join(cmd_list),
                "file": src.abspath(),
            }
        )


# ---------------------------------------------------------------------#
