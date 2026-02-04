#!/bin/bash
#This snip is not intended to be run directly!  Please copy this content
#and modify it for your specific requirements (and delete these two lines)
#
# demonstrate usage of "virtualenv"
#
# may need proxy:
#   export http_proxy="http://10.248.251.78:74"
#   export https_proxy="http://10.248.251.78:74"
#

host=$1

restore_nounset=""
test -o nounset && restore_nounset="set -u"

# install virtualenv
#[[ ${host} = mac ]]          && python3 -m venv env
[[ ${host} = ubuntu ]]       && python3 -m venv ../ossenv
#[[ ${host} = win_cygwin ]]   && python3 -m venv env
#[[ ${host} = win_gitbash ]]  && python3 -m venv env

# activate virtualenv
#
# for Windows machines, perform "which python3" and:
# 1) for "/usr/bin/python3",     use "win_cygwin"
#         ['cygwin' (tested python 3.7.4)]
# 2) for "/c/Python37/python3",  use "win_gitbash"
#         ['Git BASH/Git for Windows' with Python3 manually installed (tested Python 3.7.7)]
#
set +u
#[[ ${host} = mac ]]          && source env/bin/activate
[[ ${host} = ubuntu ]]       && source ../ossenv/bin/activate
#[[ ${host} = win_cygwin ]]   && source env/bin/activate
#[[ ${host} = win_gitbash ]]  && source env/Scripts/activate
${restore_nounset}

# manually install python3 for git bash windows virtualenv since symlinks
# do not work in git bash.
if [[ ${host} = win_gitbash ]]; then
  cp -p ../ossenv/Scripts/python.exe env/Scripts/python3.exe
fi

# these four line are not necessary:
which python
which pip
python --version
pip --version

# other optional commands:
pip list
if [[ ${host} != win_gitbash ]]; then
    # (standard command)
    pip --no-cache-dir install --upgrade pip
else
    # (non-standard command)
    python -m pip --no-cache-dir install --upgrade pip
fi
pip install pyyaml
pip install python-docx
pip install xlsxwriter
pip list

# [[... code which runs using virtualenv ...]]
PROJ=`echo $CI_COMMIT_BRANCH | sed 's/\./-/g' | sed 's/\//-/g'`
JFILE="OSS_"$PROJ".json"
python3 ./ossscan/scanoss.py $PROJ ./ $(pwd)/ossscanreport
python3 ./ossscan/scanoss_json_excel.py $(pwd)/ossscanreport/$JFILE $(pwd)/ossscanreport/

# uninstall virtualenv
set +u
#[[ ${host} = mac ]]          && deactivate
[[ ${host} = ubuntu ]]       && deactivate
#[[ ${host} = win_cygwin ]]   && deactivate
#[[ ${host} = win_gitbash ]]  && deactivate
${restore_nounset}
