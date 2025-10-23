#!/usr/bin/bash
touch $COPPELIASIM_ROOT_DIR/test-is-writeable || { echo "coppelia directory is not writable; run with --writable-tmpfs or add an overlay."; exit; }
/usr/bin/bash  --rcfile <(echo 'export PROMPT_COMMAND="history -a $HISTFILE; history -c; history -r $HISTFILE;"; export PS1="(ROS) \u@\h:\w\$ "; . ~/.bashrc') -i




