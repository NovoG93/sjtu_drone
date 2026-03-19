#!/bin/bash

# Default values
ROS_DISTRO="jazzy"
DOCKERFILE="Dockerfile"
IMAGE_NAME="georgno/sjtu_drone"
DRY_RUN=false
BUILD_ALL=false

# Supported distributions
SUPPORTED_DISTROS=("humble" "iron" "jazzy" "rolling")

usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -r <distro>   ROS 2 distribution (humble, iron, jazzy, rolling) or 'all' (default: jazzy)"
    echo "  -f <file>     Path to Dockerfile (default: Dockerfile)"
    echo "  -i <image>    Base image name (default: georgno/sjtu_drone)"
    echo "  -n            Dry run (print commands without executing)"
    echo "  -h            Show this help message"
    exit 1
}

build_image() {
    local distro=$1
    local tag="${IMAGE_NAME}:ros2-${distro}"
    
    echo "Building image for ${distro}..."
    local cmd="docker build -t ${tag} --build-arg ROS_DISTRO=${distro} -f ${DOCKERFILE} ."
    
    if [ "$DRY_RUN" = true ]; then
        echo "[DRY-RUN] $cmd"
    else
        echo "Executing: $cmd"
        if ! $cmd; then
            echo "Error: Failed to build image for ${distro}" >&2
            exit 1
        fi
        echo "Successfully built ${tag}"
    fi
}

while getopts "r:f:i:nh" opt; do
    case $opt in
        r)
            if [ "$OPTARG" == "all" ]; then
                BUILD_ALL=true
            else
                valid_distro=false
                for d in "${SUPPORTED_DISTROS[@]}"; do
                    if [ "$OPTARG" == "$d" ]; then
                        valid_distro=true
                        break
                    fi
                done
                
                if [ "$valid_distro" = false ]; then
                    echo "Error: Invalid ROS distro '$OPTARG'. Supported: ${SUPPORTED_DISTROS[*]}" >&2
                    usage
                fi
                ROS_DISTRO="$OPTARG"
            fi
            ;;
        f)
            DOCKERFILE="$OPTARG"
            ;;
        i)
            IMAGE_NAME="$OPTARG"
            ;;
        n)
            DRY_RUN=true
            ;;
        h)
            usage
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            ;;
    esac
done

# Check if Dockerfile exists
if [ ! -f "$DOCKERFILE" ]; then
    echo "Error: Dockerfile '$DOCKERFILE' not found." >&2
    exit 1
fi

if [ "$BUILD_ALL" = true ]; then
    echo "Building ALL supported distributions..."
    for distro in "${SUPPORTED_DISTROS[@]}"; do
        build_image "$distro"
    done
else
    build_image "$ROS_DISTRO"
fi
