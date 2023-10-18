## login
The `login` command allows you to authenticate your session with Citros. To use this command, you must already have a registered account with [Citros](https://citros.io), including a valid username (email) and password.

By logging in, you unlock additional features such as cloud-based simulations, data analysis tools, automated report generation, and collaboration with other Citros users. Use this command to seamlessly integrate your local workspace with the Citros platform and fully utilize its capabilities.

### parameters:
parameter|description
|--|--|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|
|`-username` | The user's username (email).|
|`-password` | The user's password|

After entering the command, if either the username or password was not given, you will be prompted for your email (the username) and password.

### example:

    $ citros login
    $ email: shalev@lulav.space
    $ Password: 
    User logged in.
