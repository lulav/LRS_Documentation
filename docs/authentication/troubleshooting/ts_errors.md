# Possible Errors

## Table of Contents
1. [Host Key Verification Failed](#host-key-verification-failed)
2. [Bad File Number](#bad-file-number)
3. [Key Already in Use](#key-already-in-use)
4. [ssh-add: Illegal Option `--apple-use-keychain`](#ssh-add-illegal-option---apple-use-keychain)
5. [SSL Certificate Problem](#ssl-certificate-problem-verify-that-the-ca-cert-is-ok)
6. [We're Doing An Ssh Key Audit](#were-doing-an-ssh-key-audit)
7. [Unknown key type](#unknown-key-type)

## Host Key Verification Failed
As a security precaution, SSH keeps track of which hosts it has previously seen.

This error means that the server to which you're connecting presented a key that doesn't match the keys seen for this server in the past.

You may see this error if the server has changed its keys unexpectedly, in which case you should be able to find an official report from a trustworthy source announcing the change. 

If you are encountering the error but can't find an official source for the server's keys, it is safest not to connect, because you may be connecting to a server other than your intended server. You may want to contact your IT department or the server's support team for help. If the server is being impersonated, the owner of the server will appreciate you informing them.

## Bad File Number
This error usually means you were unable to connect to the server. Often this is caused by firewalls and proxy servers.

When running remote Git commands or SSH, your connection might time out:

```bash
$ ssh -vT git@citros.io
> OpenSSH_8.1p1, LibreSSL 2.7.3
> debug1: Connecting to citros.io [207.97.227.239] port 22.
> debug1: connect to address 207.97.227.239 port 22: Connection timed out
> ssh: connect to host citros.io port 22: Connection timed out
> ssh: connect to host citros.io port 22: Bad file number
```

### Solving The Issue
#### Use HTTPS
Often, the simplest solution is to simply avoid SSH entirely. Most firewalls and proxies allow HTTPS traffic without issue. To take advantage of this, change the remote URL you're using:

```bash
$ git clone https://citros.io/USERNAME/REPO-NAME.git
> Cloning into 'reponame'...
> remote: Counting objects: 84, done.
> remote: Compressing objects: 100% (45/45), done.
> remote: Total 84 (delta 43), reused 78 (delta 37)
> Unpacking objects: 100% (84/84), done.
```

#### Test From a Different Network
If you can connect the computer to another network that doesn't have a firewall, you can try testing your SSH connection to CITROS. If everything works as it should, contact your network administrator for help on changing the firewall settings to allow your SSH connection to CITROS to succeed.


## Key Already in Use

This error occurs when you try to add a key that's already been added to another account or repository.

### Finding Where The Key Has Been Used
To determine where the key has already been used, open a terminal and type the `ssh` command. Use the `-i` flag to provide the path to the key you want to check:

```bash
$ ssh -T -ai ~/.ssh/id_rsa git@citros.io
# Connect to citros.io using a specific ssh key
> Hi USERNAME! You've successfully authenticated, but CITROS does not
> provide shell access.
```
The username in the response is the account on citros.io that the key is currently attached to. If the response looks something like "username/repo", the key has been attached to a repository as a [deploy key](/docs/authentication/ssh/ssh_mng_deploy_keys.md).

To force SSH to use only the key provided on the command line, use `-o` to add the `IdentitiesOnly=yes` option:

```bash
ssh -v -o "IdentitiesOnly=yes" -i ~/.ssh/id_rsa git@citros.io
```
### Fixing The Issue
To resolve the issue, first remove the key from the other account or repository and then [add it to your account](/docs/authentication/ssh/ssh_add_new.md).

If you don't have permissions to transfer the key, and can't contact a user who does, remove the keypair and [generate a brand new one](/docs/authentication/ssh/ssh_generate_key.md).

### Deploy Keys
Once a key has been attached to one repository as a deploy key, it cannot be used on another repository. If you're running into this error while setting up deploy keys, see ["Managing deploy keys"](/docs/authentication/ssh/ssh_mng_deploy_keys.md).

## ssh-add: Illegal Option `--apple-use-keychain`
This error means your version of `ssh-add` does not support macOS keychain integration, which allows you to store your passphrase in the keychain.

The `--apple-use-keychain` option is in Apple's standard version of `ssh-add`, which stores the passphrase in your keychain for you when you add an ssh key to the ssh-agent. If you have installed a different version of `ssh-add`, it may lack support for `--apple-use-keychain`.

### Solving The Issue
To add your SSH private key to the ssh-agent, you can specify the path to the Apple version of `ssh-add`:
    
    /usr/bin/ssh-add --apple-use-keychain ~/.ssh/id_ed25519

:::noteNotes:
- The `--apple-use-keychain` option is in Apple's standard version of `ssh-add`. In MacOS versions prior to Monterey (12.0), use `-K` instead of `--apple-use-keychain`.

- If you created your key with a different name, or if you are adding an existing key that has a different name, replace id_ed25519 in the command with the name of your private key file.
:::

## SSL Certificate Problem, Verify That The Ca Cert is Ok

This error means your CA root certificate is out of date. If your CA root certificate needs to be updated, you won't be able to push or pull from CITROS repositories.

The error you receive may look like the following:

```bash
$ git push -u github.main
> fatal: 'github.main' does not appear to be a git repository
> fatal: The remote end hung up unexpectedly

$ git pull -u github
> error: SSL certificate problem, verify that the CA cert is OK. Details:
> error:14090086:SSL routines:SSL3_GET_SERVER_CERTIFICATE:certificate verify failed while accessing https://citros.io/tqisjim/google-oauth.git/info/refs
> fatal: HTTP request failed
```

A "CA" is shorthand for a "certificate authority," a third-party group responsible for handling secure connections around the web. They establish digital "certificates," which are a way of ensuring that there are valid connections between two machines (like your computer and citros.io). Without a certificate, the security risk between two machines is greater.

When you receive this error, it likely means that your CA is out-of-date and needs to be updated. Generally, updating your operating system also updates your CA, and solves the problem.

## We're Doing An Ssh Key Audit
This error means the SSH key you're using to perform a Git operation is unverified.

When using an unverified key to perform Git operations, you will be prompted to perform an audit of your SSH keys.

```bash
ERROR: We're doing an SSH key audit.
Reason: unverified due to lack of use
Please visit https://citros.io/settings/ssh
to approve this key so we know it's safe.
Fingerprint: ab:08:46:83:ff:f6:c4:f8:a9:4e:68:6b:94:17:f2:46
fatal: could not read from remote repository
```

### Solving The Issue
To fix this, you need to review your SSH keys and either reject or approve the unverified key. Clicking the URL link in the error message brings you to the SSH Settings page, where the unverified SSH key is highlighted in the SSH key list.

## Unknown key type

This error means that the SSH key type you used was unrecognized or is unsupported by your SSH client.

<Tabs groupId="operating-systems">

  

<TabItem value="Mac" label="MacOS">

### About the unknown key type error
When you generate a new SSH key, you may receive an `unknown key type` error if your SSH client does not support the key type that you specify.

To solve this issue on macOS, you can update your SSH client or install a new SSH client.
### Prerequisites

You must have Homebrew installed. For more information, see the [installation guide](https://docs.brew.sh/Installation) in the Homebrew documentation.

### Solving the issue
:::cautionWarning: 

If you install OpenSSH, your computer will not be able to retrieve passphrases that are stored in the Apple keychain. You will need to enter your passphrase or interact with your hardware security key every time you authenticate with SSH to CITROS or another web service.


If you remove OpenSSH, the passphrases that are stored in your keychain will once again be retrievable. You can remove OpenSSH by entering the command brew uninstall openssh in Terminal.
:::

1. Open Terminal.
3. Enter the command `brew install openssh`.
3. Quit and relaunch Terminal.
4. Try the procedure for generating a new SSH key again. For more information, see ["Generating a new SSH key and adding it to the ssh-agent"](/docs/authentication/ssh/ssh_generate_key.md).


</TabItem>
  

<TabItem value="Linux" label="Linux">

When you generate a new SSH key, you may receive an unknown key type error if your SSH client does not support the key type that you specify.

To solve this issue on Linux, use the package manager for your Linux distribution to install a new version of OpenSSH, or compile a new version from source. If you install a different version of OpenSSH, the ability of other applications to authenticate via SSH may be affected. For more information, review the documentation for your distribution.

</TabItem>
</Tabs>



import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';