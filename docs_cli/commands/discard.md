## discard
The `discard` command allows to you discard any uncommitted changes in your Citros working directory. Simply specify the file paths of the files you would like to discard. Notice you have to specify the file paths relative to the `.citros` directory. 

If you'd like discard **all** changes in your working directory, effectively checking out the HEAD commit, instead of specifying individual file paths, you may use the --ALL flag.

**Notice**: the effects of this command cannot be undone.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|
|`files` | List of files to revert.|
|`--ALL` | Revert all files.|

### example
```bash
$ citros status
On branch main
Your branch is up to date with 'origin/main'.

Changes not staged for commit:
        modified:   project.json

$ citros discard project.json
$ citros status
On branch main
Your branch is up to date with 'origin/main'.

nothing to commit, working tree clean
```

or, using the --ALL flag:
```bash
$ citros discard --ALL
Warning: all of the following changes will be discarded:
Modified files:
   - project.json
Discard all changes? (yes/no): yes
All changes in the working directory have been reverted to the last commit.
```
