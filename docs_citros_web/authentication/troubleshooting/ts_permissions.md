# Permission Denial Issues

## Table of Contents
1. [Permission denied (publickey)](#permission-denied-publickey)
2. [Permission to user/repo denied to other-user](#permission-to-userrepo-denied-to-other-user)
3. [Permission to user/repo denied to user/other-repo](#permission-to-userrepo-denied-to-userother-repo)

## Permission Denied (Publickey)

A "Permission denied" error means that the server rejected your connection. There could be several reasons why, and the most common examples are explained below.

<Tabs groupId="operating-systems">

  
<TabItem value="Windows" label="Windows">

### Should The Sudo Command or Elevated Privileges Be Used with Git?
You should not be using the `sudo` command or elevated privileges, such as administrator permissions, with Git. If you have a very good reason you must use `sudo`, then ensure you are using it with every command (it's probably just better to use `su` to get a shell as root at that point). If you generate SSH keys without `sudo` and then try to use a command like `sudo git push`, you won't be using the same keys that you generated.

### Check That You Are Connecting to The Correct Server
Typing is hard, we all know it. Pay attention to what you type; you won't be able to connect to "githib.com" or "guthub.com". In some cases, a corporate network may cause issues resolving the DNS record as well.

To make sure you are connecting to the right domain, you can enter the following command:

```bash
$ ssh -vT git@citros.io
> OpenSSH_8.1p1, LibreSSL 2.7.3
> debug1: Reading configuration data /Users/YOU/.ssh/config
> debug1: Reading configuration data /etc/ssh/ssh_config
> debug1: /etc/ssh/ssh_config line 47: Applying options for *
> debug1: Connecting to citros.io port 22.
```

The connection should be made on port 22, unless you're overriding settings to use SSH over HTTPS.

### Always Use The "Git" User
All connections, including those for remote URLs, must be made as the "git" user. If you try to connect with your CITROS username, it will fail:

```bash 
$ ssh -T CITROS-USERNAME@citros.io
> Permission denied (publickey).
```

If your connection failed and you're using a remote URL with your CITROS username, you can change the remote URL to use the "git" user.

You should verify your connection by typing:

```bash
$ ssh -T git@citros.io
> Hi USERNAME! You've successfully authenticated...
```

### Make Sure You Have a Key That is Being Used
If you have CITROS Desktop installed, you can use it to clone repositories and not deal with SSH keys.

1. If you are using Git Bash, turn on ssh-agent:

```bash 
# start the ssh-agent in the background
$ eval "$(ssh-agent -s)"
> Agent pid 59566
```

If you are using another terminal prompt, such as Git for Windows, turn on ssh-agent:

```shell
# start the ssh-agent in the background
$ eval $(ssh-agent -s)
> Agent pid 59566
```

:::note
Note: The eval commands above start ssh-agent manually in your environment. These commands may fail if ssh-agent already runs as a background system service. If that happens, we recommend you check the relevant documentation for your environment.
:::

2. Verify that you have a private key generated and loaded into SSH.

```bash
$ ssh-add -l -E sha256
> 2048 SHA256:274ffWxgaxq/tSINAykStUL7XWyRNcRTlcST1Ei7gBQ /Users/USERNAME/.ssh/id_rsa (RSA)
```

The `ssh-add` command should print out a long string of numbers and letters. If it does not print anything, you will need to [generate a new SSH](/docs_citros_web/authentication/ssh/ssh_generate_key.md) key and associate it with CITROS.

:::tipTip: 
On most systems the default private keys (`~/.ssh/id_rsa` and `~/.ssh/identity`) are automatically added to the SSH authentication agent. You shouldn't need to run `ssh-add` path/to/key unless you override the file name when you generate a key.
:::

### Getting More Details
You can also check that the key is being used by trying to connect to git@citros.io:

```bash
$ ssh -vT git@citros.io
> ...
> debug1: identity file /Users/YOU/.ssh/id_rsa type -1
> debug1: identity file /Users/YOU/.ssh/id_rsa-cert type -1
> debug1: identity file /Users/YOU/.ssh/id_dsa type -1
> debug1: identity file /Users/YOU/.ssh/id_dsa-cert type -1
> ...
> debug1: Authentications that can continue: publickey
> debug1: Next authentication method: publickey
> debug1: Trying private key: /Users/YOU/.ssh/id_rsa
> debug1: Trying private key: /Users/YOU/.ssh/id_dsa
> debug1: No more authentication methods to try.
> Permission denied (publickey).
```
In that example, we did not have any keys for SSH to use. The "-1" at the end of the "identity file" lines means SSH couldn't find a file to use. Later on, the "Trying private key" lines also indicate that no file was found. If a file existed, those lines would be "1" and "Offering public key", respectively:

```bash
$ ssh -vT git@citros.io
> ...
> debug1: identity file /Users/YOU/.ssh/id_rsa type 1
> ...
> debug1: Authentications that can continue: publickey
> debug1: Next authentication method: publickey
> debug1: Offering RSA public key: /Users/YOU/.ssh/id_rsa
```
### Verify The Public Key is Attached to Your Account
You must provide your public key to CITROS to establish a secure connection.

1. Open the command line.

2. Start SSH agent in the background.
```bash 
$ ssh-agent -s
> Agent pid 59566
```
3. Find and take a note of your public key fingerprint.

```bash
$ ssh-add -l -E sha256
> 2048 SHA256:274ffWxgaxq/tSINAykStUL7XWyRNcRTlcST1Ei7gBQ /Users/USERNAME/.ssh/id_rsa (RSA)
```

4. In the upper-right corner of any page, click your profile photo, then click Settings.

5. In the "Access" section of the sidebar, click  SSH and GPG keys.

6. Compare the list of SSH keys with the output from the `ssh-add` command.

If you don't see your public key in CITROS, you'll need to [add your SSH key to CITROS](/docs_citros_web/authentication/ssh/ssh_add_new.md) to associate it with your computer.

:::WarningWarning: 
If you see an SSH key you're not familiar with on CITROS, delete it immediately and contact us at [contact@citros.com](mailto:contact@citros.com) for further help. An unidentified public key may indicate a possible security concern.
:::

</TabItem>

<TabItem value="Mac" label="MacOS">


### Should The Sudo Command or Elevated Privileges Be Used with Git?
You should not be using the `sudo` command or elevated privileges, such as administrator permissions, with Git. If you have a very good reason you must use `sudo`, then ensure you are using it with every command (it's probably just better to use `su` to get a shell as root at that point). If you generate SSH keys without `sudo` and then try to use a command like `sudo git push`, you won't be using the same keys that you generated.

### Check That You Are Connecting to The Correct Server
Typing is hard, we all know it. Pay attention to what you type; you won't be able to connect to "githib.com" or "guthub.com". In some cases, a corporate network may cause issues resolving the DNS record as well.

To make sure you are connecting to the right domain, you can enter the following command:

```bash
$ ssh -vT git@citros.io
> OpenSSH_8.1p1, LibreSSL 2.7.3
> debug1: Reading configuration data /Users/YOU/.ssh/config
> debug1: Reading configuration data /etc/ssh/ssh_config
> debug1: /etc/ssh/ssh_config line 47: Applying options for *
> debug1: Connecting to citros.io port 22.
```

The connection should be made on port 22, unless you're overriding settings to use SSH over HTTPS.

### Always Use The "Git" User
All connections, including those for remote URLs, must be made as the "git" user. If you try to connect with your CITROS username, it will fail:

```bash 
$ ssh -T CITROS-USERNAME@citros.io
> Permission denied (publickey).
```

If your connection failed and you're using a remote URL with your CITROS username, you can change the remote URL to use the "git" user.

You should verify your connection by typing:

```bash
$ ssh -T git@citros.io
> Hi USERNAME! You've successfully authenticated...
```

### Make Sure You Have a Key That is Being Used
If you have CITROS Desktop installed, you can use it to clone repositories and not deal with SSH keys.

1. Open Terminal

2. Verify that you have a private key generated and loaded into SSH.

```bash 
# start the ssh-agent in the background
$ eval "$(ssh-agent -s)"
> Agent pid 59566
$ ssh-add -l -E sha256
> 2048 SHA256:274ffWxgaxq/tSINAykStUL7XWyRNcRTlcST1Ei7gBQ /Users/USERNAME/.ssh/id_rsa (RSA)
```

:::note
Note: The eval commands above start ssh-agent manually in your environment. These commands may fail if ssh-agent already runs as a background system service. If that happens, we recommend you check the relevant documentation for your environment.
:::

2. Verify that you have a private key generated and loaded into SSH.

```bash
$ ssh-add -l -E sha256
> 2048 SHA256:274ffWxgaxq/tSINAykStUL7XWyRNcRTlcST1Ei7gBQ /Users/USERNAME/.ssh/id_rsa (RSA)
```

The `ssh-add` command should print out a long string of numbers and letters. If it does not print anything, you will need to [generate a new SSH](/docs_citros_web/authentication/ssh/ssh_generate_key.md) key and associate it with CITROS.

:::tipTip: 
On most systems the default private keys (`~/.ssh/id_rsa` and `~/.ssh/identity`) are automatically added to the SSH authentication agent. You shouldn't need to run `ssh-add` path/to/key unless you override the file name when you generate a key.
:::

### Getting More Details
You can also check that the key is being used by trying to connect to git@citros.io:

```bash
$ ssh -vT git@citros.io
> ...
> debug1: identity file /Users/YOU/.ssh/id_rsa type -1
> debug1: identity file /Users/YOU/.ssh/id_rsa-cert type -1
> debug1: identity file /Users/YOU/.ssh/id_dsa type -1
> debug1: identity file /Users/YOU/.ssh/id_dsa-cert type -1
> ...
> debug1: Authentications that can continue: publickey
> debug1: Next authentication method: publickey
> debug1: Trying private key: /Users/YOU/.ssh/id_rsa
> debug1: Trying private key: /Users/YOU/.ssh/id_dsa
> debug1: No more authentication methods to try.
> Permission denied (publickey).
```
In that example, we did not have any keys for SSH to use. The "-1" at the end of the "identity file" lines means SSH couldn't find a file to use. Later on, the "Trying private key" lines also indicate that no file was found. If a file existed, those lines would be "1" and "Offering public key", respectively:

```bash
$ ssh -vT git@citros.io
> ...
> debug1: identity file /Users/YOU/.ssh/id_rsa type 1
> ...
> debug1: Authentications that can continue: publickey
> debug1: Next authentication method: publickey
> debug1: Offering RSA public key: /Users/YOU/.ssh/id_rsa
```
### Verify The Public Key is Attached to Your Account
You must provide your public key to CITROS to establish a secure connection.

1. Open Terminal.

2. Start SSH agent in the background.

```bash 
$ eval "$(ssh-agent -s)"
> Agent pid 59566
```

3. Find and take a note of your public key fingerprint.

```bash
$ ssh-add -l -E sha256
> 2048 SHA256:274ffWxgaxq/tSINAykStUL7XWyRNcRTlcST1Ei7gBQ /Users/USERNAME/.ssh/id_rsa (RSA)
```

4. In the upper-right corner of any page, click your profile photo, then click Settings.

5. In the "Access" section of the sidebar, click  SSH and GPG keys.

6. Compare the list of SSH keys with the output from the `ssh-add` command.

If you don't see your public key in CITROS, you'll need to [add your SSH key to CITROS](/docs_citros_web/authentication/ssh/ssh_add_new.md) to associate it with your computer.

:::WarningWarning: 
If you see an SSH key you're not familiar with on CITROS, delete it immediately and contact us at [contact@citros.com](mailto:contact@citros.com) for further help. An unidentified public key may indicate a possible security concern.
:::


</TabItem>
  

<TabItem value="Linux" label="Linux">

### Should The Sudo Command or Elevated Privileges Be Used with Git?
You should not be using the `sudo` command or elevated privileges, such as administrator permissions, with Git. If you have a very good reason you must use `sudo`, then ensure you are using it with every command (it's probably just better to use `su` to get a shell as root at that point). If you generate SSH keys without `sudo` and then try to use a command like `sudo git push`, you won't be using the same keys that you generated.

### Check That You Are Connecting to The Correct Server
Typing is hard, we all know it. Pay attention to what you type; you won't be able to connect to "githib.com" or "guthub.com". In some cases, a corporate network may cause issues resolving the DNS record as well.

To make sure you are connecting to the right domain, you can enter the following command:

```bash
$ ssh -vT git@citros.io
> OpenSSH_8.1p1, LibreSSL 2.7.3
> debug1: Reading configuration data /Users/YOU/.ssh/config
> debug1: Reading configuration data /etc/ssh/ssh_config
> debug1: /etc/ssh/ssh_config line 47: Applying options for *
> debug1: Connecting to citros.io port 22.
```

The connection should be made on port 22, unless you're overriding settings to use SSH over HTTPS.

### Always Use The "Git" User
All connections, including those for remote URLs, must be made as the "git" user. If you try to connect with your CITROS username, it will fail:

```bash 
$ ssh -T CITROS-USERNAME@citros.io
> Permission denied (publickey).
```

If your connection failed and you're using a remote URL with your CITROS username, you can change the remote URL to use the "git" user.

You should verify your connection by typing:

```bash
$ ssh -T git@citros.io
> Hi USERNAME! You've successfully authenticated...
```

### Make Sure You Have a Key That is Being Used
If you have CITROS Desktop installed, you can use it to clone repositories and not deal with SSH keys.

1. Open Terminal

2. Verify that you have a private key generated and loaded into SSH.

```bash 
$ ssh-add -l -E sha256
> 2048 SHA256:274ffWxgaxq/tSINAykStUL7XWyRNcRTlcST1Ei7gBQ /Users/USERNAME/.ssh/id_rsa (RSA)
```

:::note
Note: The eval commands above start ssh-agent manually in your environment. These commands may fail if ssh-agent already runs as a background system service. If that happens, we recommend you check the relevant documentation for your environment.
:::

2. Verify that you have a private key generated and loaded into SSH.

```bash
$ ssh-add -l -E sha256
> 2048 SHA256:274ffWxgaxq/tSINAykStUL7XWyRNcRTlcST1Ei7gBQ /Users/USERNAME/.ssh/id_rsa (RSA)
```

The `ssh-add` command should print out a long string of numbers and letters. If it does not print anything, you will need to [generate a new SSH](/docs_citros_web/authentication/ssh/ssh_generate_key.md) key and associate it with CITROS.

:::tipTip: 
On most systems the default private keys (`~/.ssh/id_rsa` and `~/.ssh/identity`) are automatically added to the SSH authentication agent. You shouldn't need to run `ssh-add` path/to/key unless you override the file name when you generate a key.
:::

### Getting More Details
You can also check that the key is being used by trying to connect to git@citros.io:

```bash
$ ssh -vT git@citros.io
> ...
> debug1: identity file /Users/YOU/.ssh/id_rsa type -1
> debug1: identity file /Users/YOU/.ssh/id_rsa-cert type -1
> debug1: identity file /Users/YOU/.ssh/id_dsa type -1
> debug1: identity file /Users/YOU/.ssh/id_dsa-cert type -1
> ...
> debug1: Authentications that can continue: publickey
> debug1: Next authentication method: publickey
> debug1: Trying private key: /Users/YOU/.ssh/id_rsa
> debug1: Trying private key: /Users/YOU/.ssh/id_dsa
> debug1: No more authentication methods to try.
> Permission denied (publickey).
```
In that example, we did not have any keys for SSH to use. The "-1" at the end of the "identity file" lines means SSH couldn't find a file to use. Later on, the "Trying private key" lines also indicate that no file was found. If a file existed, those lines would be "1" and "Offering public key", respectively:

```bash
$ ssh -vT git@citros.io
> ...
> debug1: identity file /Users/YOU/.ssh/id_rsa type 1
> ...
> debug1: Authentications that can continue: publickey
> debug1: Next authentication method: publickey
> debug1: Offering RSA public key: /Users/YOU/.ssh/id_rsa
```
### Verify The Public Key is Attached to Your Account
You must provide your public key to CITROS to establish a secure connection.

1. Open Terminal.

2. Start SSH agent in the background.

```bash 
$ eval "$(ssh-agent -s)"
> Agent pid 59566
```

3. Find and take a note of your public key fingerprint. If you're using OpenSSH 6.7 or older:

```bash
$ ssh-add -l
> 2048 a0:dd:42:3c:5a:9d:e4:2a:21:52:4e:78:07:6e:c8:4d /Users/USERNAME/.ssh/id_rsa (RSA)
```

If you're using OpenSSH 6.8 or newer:

```bash
$ ssh-add -l -E md5
> 2048 MD5:a0:dd:42:3c:5a:9d:e4:2a:21:52:4e:78:07:6e:c8:4d /Users/USERNAME/.ssh/id_rsa (RSA)
```

4. In the upper-right corner of any page, click your profile photo, then click Settings.

5. In the "Access" section of the sidebar, click  SSH and GPG keys.

6. Compare the list of SSH keys with the output from the `ssh-add` command.

If you don't see your public key in CITROS, you'll need to [add your SSH key to CITROS](/docs_citros_web/authentication/ssh/ssh_add_new.md) to associate it with your computer.

:::WarningWarning: 
If you see an SSH key you're not familiar with on CITROS, delete it immediately and contact us at [contact@citros.com](mailto:contact@citros.com) for further help. An unidentified public key may indicate a possible security concern.
:::

</TabItem>
</Tabs>



## Permission to User/Repo Denied to Other-User

This error means the key you are pushing with is attached to an account which does not have access to the repository.

To fix this, the owner of the repository (`user`) needs to add your account (`other-user`) as a collaborator on the repository or to a team that has write access to the repository.

## Permission to User/Repo Denied to User/Other-Repo

This error means the key you are pushing with is attached to another repository as a deploy key, and does not have access to the repository you are trying to push to.

To fix this, remove the deploy key from the repository, and [add the key to your personal account](/docs_citros_web/authentication/ssh/ssh_add_new.md) instead.

If the key you are using is intended to be a deploy key, check out our guide on [deploy keys](/docs_citros_web/authentication/ssh/ssh_mng_deploy_keys.md) for more details.




import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';