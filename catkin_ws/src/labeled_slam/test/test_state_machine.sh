#!/bin/bash
echo Call invalid command
rostopic pub -1 /text_command labeled_slam/Command '{command: dance! }'
echo Switch to listening mode!
rostopic pub -1 /text_command labeled_slam/Command '{command: listen }'
echo Repeat command listen!
rostopic pub -1 /text_command labeled_slam/Command '{command: listen }'
echo Switch to driving mode from listening mode!
rostopic pub -1 /text_command labeled_slam/Command '{command: drive }'
echo Repeat command drive!
rostopic pub -1 /text_command labeled_slam/Command '{command: drive }'
echo "Try to label - should not work!"
rostopic pub -1 /text_command labeled_slam/Command '{command: label, argument: chair }'
echo "Try command go_to - should not work!"
rostopic pub -1 /text_command labeled_slam/Command '{command: go to, argument: chair  }'
echo "Switch to listening mode"
rostopic pub -1 /text_command labeled_slam/Command '{command: listen }'
echo "Label the word 'chair'"
rostopic pub -1 /text_command labeled_slam/Command '{command: label, argument: chair }'
echo "Go to label 'chair'"
rostopic pub -1 /text_command labeled_slam/Command '{command: go to, argument: chair  }'
echo "Repeat command go_to"
rostopic pub -1 /text_command labeled_slam/Command '{command: go to, argument: chair  }'
echo "Try to label word 'chair' - should not work!"
rostopic pub -1 /text_command labeled_slam/Command '{command: label, argument: chair }'
echo "Interrupt go_to state by going to drive mode"
rostopic pub -1 /text_command labeled_slam/Command '{command: drive }'
echo "Switch to listening mode"
rostopic pub -1 /text_command labeled_slam/Command '{command: listen }'
echo "Go to label 'chair'"
rostopic pub -1 /text_command labeled_slam/Command '{command: go to, argument: chair  }'
echo "Interrupt go_to state by going to listening mode"
rostopic pub -1 /text_command labeled_slam/Command '{command: listen }'
