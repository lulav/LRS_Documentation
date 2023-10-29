# Adding a New SSH Key

You can access and write data in repositories on citros.io using SSH (Secure Shell Protocol). When you connect via SSH, you authenticate using a private key file on your local machine. For more information, see ["About SSH"](/docs/authentication/ssh/ssh_overview.md).

<Tabs>

<TabItem value="web" label="Add via CITROS Account">
<Tabs groupId="operating-systems">
<TabItem value="Windows" label="Windows">

## Prerequisites

Before adding a new SSH key to your account on citros.io, complete the following steps:

1. [Check for existing SSH keys](/docs/authentication/ssh/ssh_chk_existing_key.md).

2. [Generate a new SSH](/docs/authentication/ssh/ssh_generate_key.md).

## Add SSH Key to CITROS Account

1. Copy the SSH public key to your clipboard.

```bash
$ clip < ~/.ssh/id_ed25519.pub
# Copies the contents of the id_ed25519.pub file to your clipboard
```
:::noteNotes:

- If your SSH public key file has a different name than the example code, modify the filename to match your current setup. When copying your key, don't add any newlines or whitespace.

- With Windows Subsystem for Linux (WSL), you can use `clip.exe`. Otherwise if `clip` isn't working, you can locate the hidden `.ssh` folder, open the file in your favorite text editor, and copy it to your clipboard.

- On newer versions of Windows that use the Windows Terminal, or anywhere else that uses the PowerShell command line, you may receive a `ParseError` stating that `The '&lt;' operator is reserved for future use`. In this case, the following alternative clip command should be used:

```bash
$ cat ~/.ssh/id_ed25519.pub | clip
# Copies the contents of the id_ed25519.pub file to your clipboard
```
:::

2. [Navigate to SSH Keys Settings](https://citros.io/settings?tab=ssh_keys).

3. Click "New SSH Key" button

4. Enter SSH key name.

5. Paste the public key.

6. Click "Add" button to add the SSH Key to the account.

7. The new key will be added to the list item.

</TabItem>

<TabItem value="Mac" label="MacOS">

## Prerequisites

Before adding a new SSH key to your account on citros.io, complete the following steps:

1. [Check for existing SSH keys](/docs/authentication/ssh/ssh_chk_existing_key.md).

2. [Generate a new SSH](/docs/authentication/ssh/ssh_generate_key.md).

## Add SSH Key to CITROS Account

1. Copy the SSH public key to your clipboard.

```bash
$ pbcopy < ~/.ssh/id_ed25519.pub
# Copies the contents of the id_ed25519.pub file to your clipboard
```
:::noteNotes:

- If your SSH public key file has a different name than the example code, modify the filename to match your current setup. When copying your key, don't add any newlines or whitespace.

- Alternatively, you can locate the hidden .ssh folder, open the file in your favorite text editor, and copy it to your clipboard.
:::

2. [Navigate to SSH Keys Settings](https://citros.io/settings?tab=ssh_keys).

3. Click "New SSH Key" button

4. Enter SSH key name.

5. Paste the public key.

6. Click "Add" button to add the SSH Key to the account.

7. The new key will be added to the list item.

</TabItem>

<TabItem value="Linux" label="Linux">

## Prerequisites

Before adding a new SSH key to your account on citros.io, complete the following steps:

1. [Check for existing SSH keys](/docs/authentication/ssh/ssh_chk_existing_key.md).

2. [Generate a new SSH](/docs/authentication/ssh/ssh_generate_key.md).

## Add SSH Key via CITROS Account

1. Copy the SSH public key to your clipboard.

```bash
$ clip < ~/.ssh/id_ed25519.pub
# Copies the contents of the id_ed25519.pub file to your clipboard
```
:::noteNotes:

- If your SSH public key file has a different name than the example code, modify the filename to match your current setup. When copying your key, don't add any newlines or whitespace.

- Alternatively, you can locate the hidden .ssh folder, open the file in your favorite text editor, and copy it to your clipboard.
:::

2. [Navigate to SSH Keys Settings](https://citros.io/settings?tab=ssh_keys).

3. Click "New SSH Key" button

4. Enter SSH key name.

5. Paste the public key.

6. Click "Add" button to add the SSH Key to the account.

7. The new key will be added to the list item.

</TabItem>
</Tabs>
</TabItem>

<TabItem value="local" label="Add via CITROS CLI">

## Prerequisites

1. [CITROS CLI installed](/docs_cli/overview/cli_install).

## Add SSH Key via CITROS CLI

1. Login to CITROS
    

    citros login

2. Generate and add new SSH key to CITROS


    citros setup-ssh 
    
3. Enter a name to your new ssh key 


    Please provide a descriptive title for the new ssh key (e.g. 'Personal laptop'):
    Identity added: ../.ssh/citros_ed25519 (../.ssh/citros_ed25519)
    Successfully added ssh key for 'sshExample' to Citros.


4. You can find the new SSH key in [CITROS SSH Keys Settings](https://citros.io/settings?tab=ssh_keys).



</TabItem>

</Tabs>



import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


