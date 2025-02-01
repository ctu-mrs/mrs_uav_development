# get the path to this script

PNAME=$( ps -p "$$" -o comm= )
SNAME=$( echo "$SHELL" | grep -Eo '[^/]+/?$' )

if [ "$SNAME" = "bash" ]; then
  MY_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
else
  MY_PATH=`dirname "$0"`
  MY_PATH=`( cd "$MY_PATH" && pwd )`
fi

REPO_PATH=$MY_PATH/../

# disable gitman caching
export GITMAN_CACHE_DISABLE=1

# #{ sourceShellDotfile()

getRcFile() {

  case "$SHELL" in
    *bash*)
      RCFILE="$HOME/.bashrc"
      ;;
    *zsh*)
      RCFILE="$HOME/.zshrc"
      ;;
  esac

  echo "$RCFILE"
}

sourceShellDotfile() {

  RCFILE=$( getRcFile )

  source "$RCFILE"
}

# #}
alias sb="sourceShellDotfile"

## --------------------------------------------------------------
## |                       MRS simulation                       |
## --------------------------------------------------------------

# #{ bash completion function definition
function _spawn_uav_bash_complete()
{
  local arg opts
  COMPREPLY=()
  arg="${COMP_WORDS[COMP_CWORD]}"
  opts=`rosrun mrs_simulation spawn --help | grep '  --' | awk '{print $1}'`
  COMPREPLY=( $(compgen -W "${opts}" -- ${arg}) )
}
# #}

# #{ zsh completion function definition
function _spawn_uav_zsh_complete()
{
  local opts
  reply=()
  opts=`rosrun mrs_simulation spawn --help | grep '  --' | awk '{print $1}'`
  reply=(${=opts})
}
# #}

# selection of specific function for different shells
case "$pname" in
  bash)
    complete -f "_spawn_uav_bash_complete" "spawn_uav"
    ;;
  zsh)
    compctl -k "_spawn_uav_zsh_complete" "spawn_uav"
    ;;
esac

## --------------------------------------------------------------
## |                             Git                            |
## --------------------------------------------------------------

# #{ git2https()

git2https() {

  old_remote=$(git remote get-url origin)
  echo Old remote: $old_remote

  new_remote=$(echo $old_remote | sed -r 's|.*git@(.+):(.+)|https://\1/\2|' | head -n 1)
  echo New remote: $new_remote

  if [ -n "$new_remote" ]; then
    git remote set-url origin "$new_remote"
  fi
}

# #}

# #{ git2ssh()

git2ssh() {

  old_remote=$(git remote get-url origin)
  echo Old remote: $old_remote

  new_remote=$(echo $old_remote | sed -r 's|https://([^/]+)/(.+)|git@\1:\2|' | head -n 1)
  echo New remote: $new_remote

  if [ -n "$new_remote" ]; then
    git remote set-url origin "$new_remote"
  fi
}

# #}

# #{ gitUpdateSubmodules()

gitUpdateSubmodules() {

  echo "Syncing git submodules"
  command git submodule sync
  echo "Updating git submodules"
  command git submodule update --init --recursive

  if [ -e .gitman.yml ]; then
    if [[ ! $(git status .gitman.yml --porcelain) ]]; then # if .gitman.yml is unchanged
      echo "Updating gitman sub-repos"
      gitman install
    else
      echo -e "\e[31m.gitman.yml modified, not updating sub-repos\e[0m"
    fi
  fi
}

# #}

# #{ git()

# upgrades the "git pull" to allow dotfiles profiling on linux-setup
# Other "git" features should not be changed
git() {

  case $* in

    push*)

      was_github=$(command git remote get-url origin | grep 'https://github.com' | wc -l)

      # change remote to ssh
      [ "$was_github" -ge 1 ] && git2ssh

      # run the original command
      command git "$@"
      ;;

    pull*|checkout*|"reset --hard")

      # give me the path to root of the repo we are in
      ROOT_DIR=`git rev-parse --show-toplevel` 2> /dev/null

      # we are in git repo subfolder
      if [[ "$?" == "0" ]]; then

        # if we are in the 'linux-setup' repo, use the Profile manager
        if [[ "$ROOT_DIR" == "$GIT_PATH/linux-setup" ]]; then

          PROFILE_MANAGER="$GIT_PATH/linux-setup/submodules/profile_manager/profile_manager.sh"

          bash -c "$PROFILE_MANAGER backup $GIT_PATH/linux-setup/appconfig/profile_manager/file_list.txt"

          command git "$@"

          if [[ "$?" == "0" ]]; then
            case $* in
              pull*|checkout*) # TODO: should only work for checkout of a branch
                gitUpdateSubmodules
                ;;
            esac
          fi

          if [[ "$?" == "0" ]]; then
            bash -c "$PROFILE_MANAGER deploy $GIT_PATH/linux-setup/appconfig/profile_manager/file_list.txt"
          fi

        # this is generic git repo subfolder
        else

          command git "$@"

          if [[ "$?" == "0" ]]; then
            case $* in
              pull*|checkout*) # TODO: should only work for checkout of a branch
                gitUpdateSubmodules
                ;;
            esac
          fi

        fi

      # we are not aware of being in a git subfolder
      else

        # lets run the command as it would originally would
        command git "$@"

        # and if it somehow succeeds, just update the submodules
        if [[ "$?" == "0" ]]; then
          case $* in
            pull*|checkout*) # TODO: should only work for checkout of a branch
              gitUpdateSubmodules
              ;;
          esac
        fi
      fi
      ;;
    *)
      command git "$@"
    ;;

  esac
}

# #}

# #{ gdb

gdba() {

  NODE_NAME=$1

  PID=""

  while true; do

    PID=$(ps aux | grep -e "$NODE_NAME" | vims -e "grep" "dd" | vims -s "^dWElD" | head -n 1)

    if [[ $PID != " " ]]; then
      break
    fi

    echo "waiting for '$NODE_NAME' to start"

    sleep 1

  done

  gdb attach $PID -ex c
}

# #}

alias gs="git status"
alias gcmp="git checkout master; git pull"
alias flog="~/.scripts/git-forest.sh --all --date=relative --abbrev-commit --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset' --style=15"
alias glog="git log --graph --abbrev-commit --date=relative --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset'"

## --------------------------------------------------------------
## |                         ROS aliases                        |
## --------------------------------------------------------------

# #{ presource_ros()

export ROS_PRESOURCE_PATH=/tmp/ros_presource_output.sh

presource_ros() {

  if [ -z $ROS_WORKSPACE ]; then
    echo "[presource_ros()]: \$ROS_WORKSPACE is not exported. Fill it with the path to your colcon workspace."
    return 1
  fi

  if [ ! -e ${ROS_WORKSPACE}/install ]; then
    echo "[presource_ros()]: \$ROS_WORKSPACE/install does not exist."
  fi

  # export sourced shell files rather than sourcing them directly
  export COLCON_TRACE=1
  export AMENT_TRACE_SETUP_FILES=1

  if [ -e $ROS_PRESOURCE_PATH ]; then
    rm $ROS_PRESOURCE_PATH
  fi

  source /opt/ros/jazzy/setup.zsh >> $ROS_PRESOURCE_PATH 2>&1
  [ -e $ROS_WORKSPACE/install/setup.zsh ] && source $ROS_WORKSPACE/install/setup.zsh >> $ROS_PRESOURCE_PATH 2>&1
  echo "# $ROS_WORKSPACE" >> $ROS_PRESOURCE_PATH

  source $ROS_PRESOURCE_PATH

  echo "[presource_ros()]: colcon workspace '$ROS_WORKSPACE' was presourced"
}

if [ -e ${ROS_PRESOURCE_PATH} ]; then

  # check if the workspace changed
  SAME_WS=$(cat $ROS_PRESOURCE_PATH | tail -n 1 | grep -e "# $ROS_WORKSPACE$" | wc -l)

  if [ $SAME_WS != "1" ]; then
    echo "[presource_ros()]: colcon workspace changed"
    presource_ros
  else
    if [ ! -z $TMUX ]; then
      source $ROS_PRESOURCE_PATH
    fi
  fi
else
  presource_ros
fi

# #}

# #{ colcon()

colcon() {

  CURRENT_PATH=`pwd`

  case $* in

    init*)

      if [ ! -e "build/COLCON_IGNORE" ]; then # we are NOT at the workspace root
        command colcon build # this creates a new workspace
      fi

      ;;

    build*)

      # go up the folder tree until we find the build/COLCON_IGNORE file or until we reach the root
      while [ ! -e "build/COLCON_IGNORE" ]; do
        cd ..
        if [[ `pwd` == "/" ]]; then
          # we reached the root and didn't find the build/COLCON_IGNORE file - that's a fail!
          echo "Cannot compile, probably not in a workspace (if you want to create a new workspace, call \"colcon init\" in its root first)".
          return 1
        fi
      done

      # if the flow got here, we found the build/COLCON_IGNORE file!
      # this is the folder we're looking for - call the actual colcon command here
      command colcon "$@" --symlink-install
      ret=$? # remember the return value of the colcon command
      cd "$CURRENT_PATH" # return to the path where this command was originaly called
      return $ret # return the original return value of the colcon command

      ;;

    test*)

      # go up the folder tree until we find the build/COLCON_IGNORE file or until we reach the root
      while [ ! -e "build/COLCON_IGNORE" ]; do
        cd ..
        if [[ `pwd` == "/" ]]; then
          # we reached the root and didn't find the build/COLCON_IGNORE file - that's a fail!
          echo "Cannot run tests, probably not in a workspace (if you want to create a new workspace, call \"colcon init\" in its root first)".
          return 1
        fi
      done

      # if the flow got here, we found the build/COLCON_IGNORE file!
      # this is the folder we're looking for - call the actual colcon command here
      command colcon "$@"
      ret=$? # remember the return value of the colcon command
      cd "$CURRENT_PATH" # return to the path where this command was originaly called
      return $ret # return the original return value of the colcon command

      ;;

    clean*)

      if [ -e "build/COLCON_IGNORE" ]; then # we are at the workspace root
        rm -r build install log
        mkdir build
        cd build
        touch COLCON_IGNORE
      else
        while [ ! -e "build/COLCON_IGNORE" ]; do
          cd ..

          if [[ `pwd` == "/" ]]; then
            echo "Cannot clean, not in a workspace!"
            break
          elif [ -e "build/COLCON_IGNORE" ]; then
            rm -r build install log
            mkdir build
            cd build
            touch COLCON_IGNORE
            break
          fi
        done
      fi

      cd "$CURRENT_PATH" # return to the original folder where the command was called

      ;;

    *)
      command colcon $@
      ;;

  esac
}

# #}

# #{ cb()

cb()  {

  colcon build "$@"
}

# #}

# #{ roscd()

roscd() {

  if [ -z $ROS_WORKSPACE ]; then
    echo "[roscd]: \$ROS_WORKSPACE is not exported. Fill it with the path to your colcon workspace."
    return 1
  fi

  packages=$(colcon list --base-paths $ROS_WORKSPACE 2>/dev/null)

  package_path=$(echo $packages | grep -E "^$1\s" | awk '{print $2}')

  if [ ! -z $package_path ]; then
    cd $package_path
  else
    cd $ROS_WORKSPACE/src
  fi
}

_roscd_complete() {

  local current_word
  current_word="${COMP_WORDS[COMP_CWORD]}"

  # Get the list of package names using colcon list
  local packages
  packages=$(colcon list --base-paths ${ROS_WORKSPACE} 2>/dev/null)

  # reply=(${=packages})
  COMPREPLY=($(compgen -W "$packages" -- "$current_word"))
}

complete -F _roscd_complete roscd

# #}

# #}
