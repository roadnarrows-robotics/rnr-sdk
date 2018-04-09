#!/bin/bash
#
# Permanently delete large files(s) from git repository history.
#
# use  "git lola --name-status" command to find files
# 
# See:
# http://stackoverflow.com/questions/2100907/how-to-remove-delete-a-large-file-from-commit-history-in-git-repository
#

# change this to file to delete
todelfile="hekateros_moveit_4dof/default_warehouse_mongo_db/journal/prealloc.2"

git filter-branch --prune-empty -d /dev/shm/scratch \
  --index-filter "git rm --cached -f --ignore-unmatch ${todelfile}" \
  --tag-name-filter cat -- --all
