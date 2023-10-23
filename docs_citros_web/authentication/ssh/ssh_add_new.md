# Adding a new SSH key

To configure your account on GitHub.com to use your new (or existing) SSH key, you'll also need to add the key to your account.

<Tabs groupId="operating-systems">

<TabItem value="Windows" label="Windows">
     
## About addition of SSH keys to your account

You can access and write data in repositories on GitHub.com using SSH (Secure Shell Protocol). When you connect via SSH, you authenticate using a private key file on your local machine. For more information, see ["About SSH"](/docs_citros_web/authentication/ssh/ssh_overview.md).

## Prerequisites

Before adding a new SSH key to your account on citros.com, complete the following steps:

1. Check for existing SSH keys. For more information, see ["Checking for existing SSH keys"](/docs_citros_web/authentication/ssh/ssh_chk_existing_key.md).

2. Generate a new SSH key and add it to your machine's SSH agent. For more information, see ["Generate key"](/docs_citros_web/authentication/ssh/ssh_generate_key.md).

## Adding a new SSH key to your account

You can add an SSH key and use it for authentication, or commit signing, or both. If you want to use the same SSH key for both authentication and signing, you need to upload it twice.

After adding a new SSH authentication key to your account on citros.com, you can reconfigure any local repositories to use SSH. 

1. Copy the SSH public key to your clipboard.

If your SSH public key file has a different name than the example code, modify the filename to match your current setup. When copying your key, don't add any newlines or whitespace.

```bash
$ clip < ~/.ssh/id_ed25519.pub
# Copies the contents of the id_ed25519.pub file to your clipboard
```
:::noteNotes:

- With Windows Subsystem for Linux (WSL), you can use `clip.exe`. Otherwise if `clip` isn't working, you can locate the hidden `.ssh` folder, open the file in your favorite text editor, and copy it to your clipboard.

- On newer versions of Windows that use the Windows Terminal, or anywhere else that uses the PowerShell command line, you may receive a `ParseError` stating that `The '&lt;' operator is reserved for future use`. In this case, the following alternative clip command should be used:

```bash
$ cat ~/.ssh/id_ed25519.pub | clip
# Copies the contents of the id_ed25519.pub file to your clipboard
```
:::

2. In the upper-right corner of any page, click your profile photo, then click Settings.

3. In the "Access" section of the sidebar, click  SSH and GPG keys.

4. Click New SSH key or Add SSH key.

5. In the "Title" field, add a descriptive label for the new key. For example, if you're using a personal laptop, you might call this key "Personal laptop".

6. Select the type of key, either authentication or signing.

7. In the "Key" field, paste your public key.

8. Click Add SSH key.

9. If prompted, confirm access to your account on CITROS. 



</TabItem>
  

<TabItem value="Mac" label="MacOS">
## About addition of SSH keys to your account

You can access and write data in repositories on GitHub.com using SSH (Secure Shell Protocol). When you connect via SSH, you authenticate using a private key file on your local machine. For more information, see ["About SSH"](/docs_citros_web/authentication/ssh/ssh_overview.md).

## Prerequisites

Before adding a new SSH key to your account on citros.com, complete the following steps:

1. Check for existing SSH keys. For more information, see ["Checking for existing SSH keys"](/docs_citros_web/authentication/ssh/ssh_chk_existing_key.md).

2. Generate a new SSH key and add it to your machine's SSH agent. For more information, see ["Generate key"](/docs_citros_web/authentication/ssh/ssh_generate_key.md).

## Adding a new SSH key to your account

You can add an SSH key and use it for authentication, or commit signing, or both. If you want to use the same SSH key for both authentication and signing, you need to upload it twice.

After adding a new SSH authentication key to your account on citros.com, you can reconfigure any local repositories to use SSH. 

1. Copy the SSH public key to your clipboard.

If your SSH public key file has a different name than the example code, modify the filename to match your current setup. When copying your key, don't add any newlines or whitespace.

```bash
$ pbcopy < ~/.ssh/id_ed25519.pub
# Copies the contents of the id_ed25519.pub file to your clipboard
```
:::tipTIP:

If pbcopy isn't working, you can locate the hidden .ssh folder, open the file in your favorite text editor, and copy it to your clipboard.
:::

2. In the upper-right corner of any page, click your profile photo, then click Settings.

3. In the "Access" section of the sidebar, click  SSH and GPG keys.

4. Click New SSH key or Add SSH key.

5. In the "Title" field, add a descriptive label for the new key. For example, if you're using a personal laptop, you might call this key "Personal laptop".

6. Select the type of key, either authentication or signing.

7. In the "Key" field, paste your public key.

8. Click Add SSH key.

9. If prompted, confirm access to your account on CITROS. 

</TabItem>
  

<TabItem value="Linux" label="Linux">

## About addition of SSH keys to your account

You can access and write data in repositories on GitHub.com using SSH (Secure Shell Protocol). When you connect via SSH, you authenticate using a private key file on your local machine. For more information, see ["About SSH"](/docs_citros_web/authentication/ssh/ssh_overview.md).

## Prerequisites

Before adding a new SSH key to your account on citros.com, complete the following steps:

1. Check for existing SSH keys. For more information, see ["Checking for existing SSH keys"](/docs_citros_web/authentication/ssh/ssh_chk_existing_key.md).

2. Generate a new SSH key and add it to your machine's SSH agent. For more information, see ["Generate key"](/docs_citros_web/authentication/ssh/ssh_generate_key.md).

## Adding a new SSH key to your account

You can add an SSH key and use it for authentication, or commit signing, or both. If you want to use the same SSH key for both authentication and signing, you need to upload it twice.

After adding a new SSH authentication key to your account on citros.com, you can reconfigure any local repositories to use SSH. 

1. Copy the SSH public key to your clipboard.

If your SSH public key file has a different name than the example code, modify the filename to match your current setup. When copying your key, don't add any newlines or whitespace.

```bash
$ clip < ~/.ssh/id_ed25519.pub
# Copies the contents of the id_ed25519.pub file to your clipboard
```
:::tipTip:

Alternatively, you can locate the hidden .ssh folder, open the file in your favorite text editor, and copy it to your clipboard.
:::

2. In the upper-right corner of any page, click your profile photo, then click Settings.

3. In the "Access" section of the sidebar, click  SSH and GPG keys.

4. Click New SSH key or Add SSH key.

5. In the "Title" field, add a descriptive label for the new key. For example, if you're using a personal laptop, you might call this key "Personal laptop".

6. Select the type of key, either authentication or signing.

7. In the "Key" field, paste your public key.

8. Click Add SSH key.

9. If prompted, confirm access to your account on CITROS. 

</TabItem>
</Tabs>




import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';