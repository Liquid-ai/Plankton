process_dir() {
    for d in $1/*; do
    if [ -d "$d" ]; then
        if [ -f "$d/package.xml" ]; then
            (cd $d && sed -i "/^\w*#/d" CMakeLists.txt && python3 /home/slock/Documents/catkin-to-ament.py)
        fi
        process_dir $d
    fi
    done
}

process_dir .