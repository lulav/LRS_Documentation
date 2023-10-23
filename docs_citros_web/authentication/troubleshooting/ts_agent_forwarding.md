# Agent Forwarding Troubleshooting 

Here are some things to look out for when troubleshooting SSH agent forwarding.

You must be using an SSH URL to check out code
SSH forwarding only works with SSH URLs, not HTTP(s) URLs. Check the .git/config file on your server and ensure the URL is an SSH-style URL like below:

    [remote "origin"]
    url = git@github.com:YOUR_ACCOUNT/YOUR_PROJECT.git
    fetch = +refs/heads/*:refs/remotes/origin/*

## Your SSH keys must work locally

Before you can make your keys work through agent forwarding, they must work locally first. [Our guide on generating SSH keys](../ssh/ssh_generate_key.md) can help you set up your SSH keys locally.

## Your system must allow SSH agent forwarding
Sometimes, system configurations disallow SSH agent forwarding. You can check if a system configuration file is being used by entering the following command in the terminal:

```bash
    $ ssh -v URL
# Connect to the specified URL with verbose debug output
> OpenSSH_8.1p1, LibreSSL 2.7.3</span>
> debug1: Reading configuration data /Users/YOU/.ssh/config
> debug1: Applying options for example.com
> debug1: Reading configuration data /etc/ssh_config
> debug1: Applying options for *
$ exit
# Returns to your local command prompt
```

In the example above, the file `~/.ssh/config` is loaded first, then `/etc/ssh_config` is read. We can inspect that file to see if it's overriding our options by running the following commands:

```bash
$ cat /etc/ssh_config
# Print out the /etc/ssh_config file
> Host *
>   SendEnv LANG LC_*
>   ForwardAgent no
```

In this example, our /etc/ssh_config file specifically says ForwardAgent no, which is a way to block agent forwarding. Deleting this line from the file should get agent forwarding working once more.

## Your server must allow SSH agent forwarding on inbound connections
Agent forwarding may also be blocked on your server. You can check that agent forwarding is permitted by SSHing into the server and running `sshd_config`. The output from this command should indicate that `AllowAgentForwarding` is set.

## Your local ssh-agent must be running
On most computers, the operating system automatically launches ssh-agent for you. On Windows, however, you need to do this manually. We have [a guide on how to start `ssh-agent` whenever you open Git Bash](../ssh/ssh_passphrases.md).

To verify that ssh-agent is running on your computer, type the following command in the terminal:

```bash
$ echo "$SSH_AUTH_SOCK"
# Print out the SSH_AUTH_SOCK variable
> /tmp/launch-kNSlgU/Listeners
```

## Your key must be available to `ssh-agent`
You can check that your key is visible to ssh-agent by running the following command:

    ssh-add -L

If the command says that no identity is available, you'll need to add your key:

    ssh-add YOUR-KEY

:::note[Note]:
- On macOS, ssh-agent will "forget" this key, once it gets restarted during reboots. But you can import your SSH keys into Keychain using this command:
    ssh-add --apple-use-keychain YOUR-KEY


- The `--apple-use-keychain` option stores the passphrase in your keychain for you when you add an SSH key to the ssh-agent. If you chose not to add a passphrase to your key, run the command without the `--apple-use-keychain` option.
The `--apple-use-keychain` option is in Apple's standard version of `ssh-add`. In MacOS versions prior to Monterey (12.0), the `--apple-use-keychain` and --apple-load-keychain flags used the syntax `-K` and `-A`, respectively.
If you don't have Apple's standard version of ssh-add installed, you may receive an error. For more information, see "Error: ssh-add: illegal option `--apple-use-keychai`n."
If you continue to be prompted for your passphrase, you may need to add the command to your `~/.zshrc` file (or your `~/.bashrc` file for bash).

:::