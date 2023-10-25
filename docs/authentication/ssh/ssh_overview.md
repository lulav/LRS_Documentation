# SSH Overview

Using the SSH protocol, you can connect and authenticate to remote servers and services. With SSH keys, you can connect to CITROS without supplying your username and personal access token at each visit. You can also use an SSH key to sign commits.

You can access and write data in repositories on citros.io using SSH (Secure Shell Protocol). When you connect via SSH, you authenticate using a private key file on your local machine. For more information about SSH, see [Secure Shell](https://en.wikipedia.org/wiki/Secure_Shell) on Wikipedia.

When you set up SSH, you will need to generate a new private SSH key and add it to the SSH agent. You must also add the public SSH key to your account on GitHub before you use the key to authenticate or sign commits. For more information, see [Generate SSH Key](/docs/authentication/ssh/ssh_generate_key.md).

You can further secure your SSH key by using a hardware security key, which requires the physical hardware security key to be attached to your computer when the key pair is used to authenticate with SSH. You can also secure your SSH key by adding your key to the ssh-agent and using a passphrase. For more information, see 
[SSH key passphrases](/docs/authentication/ssh/ssh_passphrases.md).

To maintain account security, you can regularly review your SSH keys list and delete any keys that are invalid or have been compromised. For more information, see [SSH Screen](docs/authentication/account/profile/pr_ssh.md).