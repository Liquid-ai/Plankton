process_dir() {
    for d in $1/*; do
    if [ -d "$d" ]; then
        if [ -f "$d/package.xml" ]; then
            if grep -Fxq "# Add support for C++11" "$d/CMakeLists.txt"
            then
                echo "$d was already processed, skipping"
            else
                echo "processing $d"
                (cd $d && sed -i "/^\w*#/d" CMakeLists.txt && python3 /home/slock/Documents/catkin-to-ament.py)
            fi
        fi
        process_dir $d
    fi
    done
}

process_dir .