# Agent Forwarding

To simplify deploying to a server, you can set up SSH agent forwarding to securely use local SSH keys.

SSH agent forwarding can be used to make deploying to a server simple. It allows you to use your local SSH keys instead of leaving keys (without passphrases!) sitting on your server.

If you've already set up an SSH key to interact with GitHub, you're probably familiar with `ssh-agent`. It's a program that runs in the background and keeps your key loaded into memory, so that you don't need to enter your passphrase every time you need to use the key. The nifty thing is, you can choose to let servers access your local `ssh-agent` as if they were already running on the server. This is sort of like asking a friend to enter their password so that you can use their computer.

Check out [Steve Friedl's Tech Tips guide](http://www.unixwiz.net/techtips/ssh-agent-forwarding.html) for a more detailed explanation of SSH agent forwarding.

## Setting up SSH agent forwarding
Ensure that your own SSH key is set up and working. You can use [our guide on generating SSH keys](/docs/authentication/ssh/ssh_generate_key.md) if you've not done this yet.

You can test that your local key works by entering ssh -T git@citros.com in the terminal:

    $ ssh -T git@citros.com
    # Attempt to SSH in to github
    > Hi USERNAME! You've successfully authenticated, but GitHub does not provide
    > shell access.

We're off to a great start. Let's set up SSH to allow agent forwarding to your server.
1. Using your favorite text editor, open up the file at `~/.ssh/config`. If this file doesn't exist, you can create it by entering touch `~/.ssh/config` in the terminal.
2. Enter the following text into the file, replacing example.com with your server's domain name or IP:

```bash
Host example.com
ForwardAgent yes
```

 :Warning: You may be tempted to use a wildcard like `Host *` to just apply this setting to all SSH connections. That's not really a good idea, as you'd be sharing your local SSH keys with every server you SSH into. They won't have direct access to the keys, but they will be able to use them as you while the connection is established. You should only add servers you trust and that you intend to use with agent forwarding.

## Testing SSH agent forwarding

To test that agent forwarding is working with your server, you can SSH into your server and run `ssh -T git@github.com` once more. If all is well, you'll get back the same prompt as you did locally.

If you're unsure if your local key is being used, you can also inspect the `SSH_AUTH_SOCK` variable on your server:

```bash
$ echo "$SSH_AUTH_SOCK"
# Print out the SSH_AUTH_SOCK variable
> /tmp/ssh-4hNGMk8AZX/agent.79453
```
If the variable is not set, it means that agent forwarding is not working:

```bash 
$ echo "$SSH_AUTH_SOCK"
# Print out the SSH_AUTH_SOCK variable
> [No output]
$ ssh -T git@github.com
# Try to SSH to github
> Permission denied (publickey).
```

In any issue, checkout [Agent Forwarding Troubleshooting](/docs/authentication/troubleshooting/ts_agent_forwarding.md).