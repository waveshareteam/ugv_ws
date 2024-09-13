#!/bin/bash

# Function to start the container
enter_container() {
    echo "Entering the container..."
    docker start ugv_rpi_ros_humble
    if [ $? -eq 0 ]; then
        echo "Container started successfully."
        echo "Executing docker exec command to open a bash shell in the container..."
        docker exec -it ugv_rpi_ros_humble /bin/bash -c "/home/ws/ugv_ws/remotessh.sh"
        if [ $? -eq 0 ]; then
            echo "Opened bash shell in the container."
            exit
        else
            echo "Failed to open bash shell in the container."
        fi
    else
        echo "Failed to enter the container, please check the error."
    fi
}

# Main menu function
main_menu() {
    while true; do
        echo "Please choose an option:"
        echo "1. Enter container"
        read -p "Enter the option number and press Enter: " choice
        
        case $choice in
            1)
                enter_container
                ;;
            *)
                echo "Invalid option, please try again."
                ;;
        esac
    done
}

# Call the main menu function
main_menu
