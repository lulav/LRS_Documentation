# Testing your SSH connection

After you've set up your SSH key and added it to your account on citros.io, you can test your connection.

<Tabs groupId="operating-systems">
  
<TabItem value="Windows" label="Windows">

Before testing your SSH connection, you should have:

- [Checked for existing SSH keys](./ssh_chk_existing_key.md)
- [Generated a new SSH key](./ssh_generate_key.md)
- [Added a new SSH key](./ssh_add_new.md)

When you test your connection, you'll need to authenticate this action using your password, which is the SSH key passphrase you created earlier. For more information on working with SSH key passphrases, see ["Working with SSH key passphrases"](./ssh_passphrases.md).

1. Open Bash.

2. Enter the following:

```bash
$ ssh -T git@citros.io
# Attempts to ssh to CITROS
```

You may see a warning like this:

```bash
> The authenticity of host 'citros.io (IP ADDRESS)' can't be established.
> ED25519 key fingerprint is SHA256:+DiY3wvvV6TuJJhbpZisF/zLDA0zPMSvHdkr4UvCOqU.
> Are you sure you want to continue connecting (yes/no)?
```
3. Verify that the fingerprint in the message you see matches CITROS's public key fingerprint. If it does, then type yes:

```bash
hello USERNAME, this is git@gitolite-gitolite-5f69568574-gd54j running gitolite3 3.6.12 on git 2.40.1
PERMISSIONS	ORGANIZATION/REPOSITORY
```
:::noteNote: 
The remote command should exit with code 1.
:::

4. Verify that the resulting message contains your username. If you receive a "permission denied" message, see "Error: Permission denied (publickey)."

</TabItem>
  

<TabItem value="Mac" label="MacOS">

Before testing your SSH connection, you should have:

- [Checked for existing SSH keys](./ssh_chk_existing_key.md)
- [Generated a new SSH key](./ssh_generate_key.md)
- [Added a new SSH key](./ssh_add_new.md)

When you test your connection, you'll need to authenticate this action using your password, which is the SSH key passphrase you created earlier. For more information on working with SSH key passphrases, see ["Working with SSH key passphrases"](./ssh_passphrases.md).

1. Open Terminal.

2. Enter the following:

```bash
$ ssh -T git@citros.io
# Attempts to ssh to CITROS
```

You may see a warning like this:

```bash
> The authenticity of host 'citros.io (IP ADDRESS)' can't be established.
> ED25519 key fingerprint is SHA256:+DiY3wvvV6TuJJhbpZisF/zLDA0zPMSvHdkr4UvCOqU.
> Are you sure you want to continue connecting (yes/no)?
```
3. Verify that the fingerprint in the message you see matches CITROS's public key fingerprint. If it does, then type yes:

```bash
hello USERNAME, this is git@gitolite-gitolite-5f69568574-gd54j running gitolite3 3.6.12 on git 2.40.1
PERMISSIONS	ORGANIZATION/REPOSITORY
```
:::noteNote: 
The remote command should exit with code 1.
:::

4. Verify that the resulting message contains your username. If you receive a "permission denied" message, see "Error: Permission denied (publickey)."

</TabItem>


<TabItem value="Linux" label="Linux">

Before testing your SSH connection, you should have:

- [Checked for existing SSH keys](./ssh_chk_existing_key.md)
- [Generated a new SSH key](./ssh_generate_key.md)
- [Added a new SSH key](./ssh_add_new.md)

When you test your connection, you'll need to authenticate this action using your password, which is the SSH key passphrase you created earlier. For more information on working with SSH key passphrases, see ["Working with SSH key passphrases"](./ssh_passphrases.md).

1. Open Terminal.

2. Enter the following:

```bash
$ ssh -T git@citros.io
# Attempts to ssh to CITROS
```

You may see a warning like this:

```bash
> The authenticity of host 'citros.io (IP ADDRESS)' can't be established.
> ED25519 key fingerprint is SHA256:+DiY3wvvV6TuJJhbpZisF/zLDA0zPMSvHdkr4UvCOqU.
> Are you sure you want to continue connecting (yes/no)?
```
3. Verify that the fingerprint in the message you see matches CITROS's public key fingerprint. If it does, then type yes:

```bash
hello USERNAME, this is git@gitolite-gitolite-5f69568574-gd54j running gitolite3 3.6.12 on git 2.40.1
PERMISSIONS	ORGANIZATION/REPOSITORY
```
:::noteNote: 
The remote command should exit with code 1.
:::

4. Verify that the resulting message contains your username. If you receive a "permission denied" message, see "Error: Permission denied (publickey)."


</TabItem>
</Tabs>


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';