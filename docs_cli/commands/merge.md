## merge
The `merge` command enables you to integrate another branch into your current one. You'll be shown a list of accessible branches to select from. When both your branch and the target branch have modifications to the same file, the outcome varies based on the nature of these changes. Non-conflicting changes will be seamlessly merged. 

However, if conflicts arise, the merge operation halts, requiring you to address these discrepancies manually, using a diff/merge tool. 

### example

In the following example we attempt to merge the branch `master` into the current branch:

    $ citros merge
    ? Please choose the branch you wish to merge into the current branch: master
    Merge failed due to conflicting changes between the current branch and `master`.
    Files with conflicts:
    - notebooks/test.ipynb
    Please resolve the conflicts manually.
    ...

If you are not running inside a dev-container, an instance of VS-Code will be automatically opened (pending your aproval) for you to use a merge tool. After all conflicts have been resolved, save the files, close VS-Code and answer `y` to indicate that all conflicts have indeed been resolved. At this point Citros will commit the merge on your behalf.

If you are running inside a dev-container, you'll have to run a few git commands by yourself, but not to worry - Citros will provide you with step-by-step instructions:


    $ citros merge
    ? Please choose the branch you wish to merge into the current branch: test_branch
    Merge failed due to conflicting changes between the current branch and `test_branch`.
    Files with conflicts:
    - parameter_setups/functions/my_func.py
    Please resolve the conflicts manually.
    Since you are running inside a dev-container, you'll have to:
    1. Open a terminal, e.g.
    ctrl-alt-t
    2. Navigate to the .citros directory under your project, e.g.
    cd path/to/your/project/.citros
    3. Run the following two commands to set VS code as the git merge tool for your .citros repo:
    git config merge.tool code
    git config mergetool.code.cmd "code --wait $MERGED"
    (if you already have a merge tool set for git, you may skip this step).
    4. Open your mergetool (i.e. VS code) to resolve the conflict:
    git mergetool

    After all conflicts have been resolved, save the files, close the merge tool, answer y in the terminal and close it.
    Press y to commit the merge or n to abort the merge.
    Note: if you press y and there are still unresolved conflicts, the merge will still be aborted.
    All conflicts resolved (y/n): y
    Conflicts resolved. Committing the merge...


**Note:** For files that Citros manages, like `project.json`, conflicts will be auto-resolved in favor of the current branch's version.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|