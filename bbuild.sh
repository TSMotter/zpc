#!/usr/bin/env bash

# Global flags
global_flag_f_format=0
global_flag_b_build=0
global_flag_r_rebuild=0
global_flag_l_flash=0
global_flag_v_verbose=0

#################################################################################

#
# Print usage.
#
function print_help()
{
    cat << EOF
    Use like this:

    ./bbuild.sh <flags>

    flags:
    -f, --format        [f]ormat all source files with clang-formatter
    -b, --build         [b]uild
    -r, --rebuild       [r]euild
    -l, --flash         f[l]ash binary to target
    -v, --verbose       [v]erbose

EOF

    return 0
}

################################################################################

#
# Print a fancy banner to separate sections visually.
#
function print_banner()
{
    if [[ global_flag_v_verbose -eq 1 ]]; then
        printf "\n================================================================================\n"
        printf "%s\n" "$*"
        printf "================================================================================\n\n"
    fi
}

################################################################################

#
# Print text a simple header to make it easier to spot.
#
function print_header()
{
    if [[ global_flag_v_verbose -eq 1 ]]; then
        printf "==-- %s --==\n" "$*"
    fi
}

################################################################################

#
# Format with clang-format
#
function func_format()
{
    print_banner "Formatting code"

    # -iname pattern: Returns true if the file’s name matches the provided shell 
    # pattern. The matching here is case insensitive.

    # xargs is a great command that reads streams of data from standard input, 
    # then generates and executes command lines; meaning it can take output of a 
    # command and passes it as argument of another command. If no command is specified, 
    # xargs executes echo by default. You many also instruct it to read data from a file 
    # instead of stdin.

    find . -iname '*.c' | grep --invert-match './build' | grep --invert-match 'external' | xargs clang-format -i -style=file

    if find . -iname '*.h' | grep --invert-match './build' | grep --invert-match 'external'; then
        find . -iname '*.h' | grep --invert-match './build' | grep --invert-match 'external' | xargs clang-format -i -style=file
    fi

    python -m autopep8 -i -a -a -a scripts/stream_sin_wave.py
}

################################################################################

#
# Build with west
#
function func_build()
{
    print_banner "Building code"

    west build --pristine
}

################################################################################

#
# Rebuild with make/cmake
#
function func_rebuild()
{
    print_banner "Deleting build directory"

    rm -rf build
    func_build
}

################################################################################

#
# Flash the binary to the target
#
function func_flash()
{
    print_banner "Flashing code"

    west flash
}

################################################################################

#
# Gather all params passed to the script
#
function gather_params()
{
    while [[ $# -gt 0 ]]; do
        case $1 in
            -f | --format)
                global_flag_f_format=1
                shift
                ;;
            -b | --build)
                global_flag_b_build=1
                shift
                ;;
            -r | --rebuild)
                global_flag_r_rebuild=1
                shift
                ;;
            -l | --flash)
                global_flag_l_flash=1
                shift
                ;;
            -v | --verbose)
                global_flag_v_verbose=1
                shift
                ;;
            -h | --help)
                print_help
                exit
                ;;
        esac
    done
}

################################################################################

#
# Execute script logic based on parameters
#
function execute_logic()
{
    if [[ global_flag_f_format -eq 1 ]]; then
        func_format
    fi
    if [[ global_flag_b_build -eq 1 ]]; then
        func_build
    fi
    if [[ global_flag_r_rebuild -eq 1 ]]; then
        func_rebuild
    fi
    if [[ global_flag_l_flash -eq 1 ]]; then
        func_flash
    fi
}

################################################################################

#
# main function
#
function main()
{
    # Gather params
    gather_params "$@"
    execute_logic
}


main "$@"