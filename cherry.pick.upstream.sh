# Copyright (C) 2014 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

function get_commit_message() {
  local repo_url=$1
  local revision=$2
  echo "`svn log $repo_url -r$revision`
(cherry-picked from $repo_url@$revision)"
}

function usage() {
  echo "Usage: $0 (vex|val) <revision number>"
  exit 1;
}

valgrind_svn_url="svn://svn.valgrind.org/valgrind/trunk"
vex_svn_url="svn://svn.valgrind.org/vex/trunk"

current_dir=`realpath \`dirname $0\``
valgrind_dir=$current_dir/main
vex_dir=$valgrind_dir/VEX

if [ "$1" = "" -o "$2" = "" ]; then
  usage
fi

if [ "$1" = "vex" ]; then
  repo_url=$vex_svn_url
  working_dir=$vex_dir
elif [ "$1" = "val" ]; then
  repo_url=$valgrind_svn_url
  working_dir=$valgrind_dir
else
  usage
fi

revision=$2
revision_minus_one=$[ $revision - 1 ]

commit_message=$(get_commit_message $repo_url $revision)


echo "Cherry-picking from $repo_url r$revision ... (in $working_dir)" | tee $current_dir/cherry-pick-$revision.log
cd $working_dir
svn diff -r$revision_minus_one:$revision $repo_url | tee $current_dir/cherry-pick-patch-$revision.txt | patch -Ep0 | tee -a $current_dir/herry-pick-$revision.log

git commit -a -m "$commit_message"
