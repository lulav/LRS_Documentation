# Checking for existing SSH keys

Before you generate a new SSH key, you should check your local machine for existing keys.

<Tabs groupId="operating-systems">

<TabItem value="Windows" label="Windows">

1. Open Git Bash.

2. Enter `ls -al ~/.ssh` to see if existing SSH keys are present.

```bash
$ ls -al ~/.ssh
# Lists the files in your .ssh directory, if they exist
```
3. Check the directory listing to see if you already have a public SSH key. By default, the filenames of supported public keys for GitHub are one of the following.
- id_rsa.pub
- id_ecdsa.pub
- id_ed25519.pub

:::tipTip: If you receive an error that ~/.ssh doesn't exist, you do not have an existing SSH key pair in the default location. You can create a new SSH key pair in the next step.
:::

4. Either generate a new SSH key or upload an existing key.

- If you don't have a supported public and private key pair, or don't wish to use any that are available, generate a new SSH key.

- If you see an existing public and private key pair listed (for example, id_rsa.pub and id_rsa) that you would like to use to connect to GitHub, you can add the key to the ssh-agent.

For more information about generation of a new SSH key or addition of an existing key to the ssh-agent, see ["Generating a new SSH key and adding it to the ssh-agent."](./ssh_generate_key.md).

</TabItem>

<TabItem value="Mac" label="MacOS">

1. Open Terminal.

2. Enter `ls -al ~/.ssh` to see if existing SSH keys are present.

```bash
$ ls -al ~/.ssh
# Lists the files in your .ssh directory, if they exist
```
3. Check the directory listing to see if you already have a public SSH key. By default, the filenames of supported public keys for GitHub are one of the following.
- id_rsa.pub
- id_ecdsa.pub
- id_ed25519.pub

:::tipTip: If you receive an error that ~/.ssh doesn't exist, you do not have an existing SSH key pair in the default location. You can create a new SSH key pair in the next step.
:::

4. Either generate a new SSH key or upload an existing key.

- If you don't have a supported public and private key pair, or don't wish to use any that are available, generate a new SSH key.

- If you see an existing public and private key pair listed (for example, id_rsa.pub and id_rsa) that you would like to use to connect to GitHub, you can add the key to the ssh-agent.

For more information about generation of a new SSH key or addition of an existing key to the ssh-agent, see ["Generating a new SSH key and adding it to the ssh-agent."](./ssh_generate_key.md).

</TabItem>



<TabItem value="Linux" label="Linux">

1. Open Terminal.

2. Enter `ls -al ~/.ssh` to see if existing SSH keys are present.

```bash
$ ls -al ~/.ssh
# Lists the files in your .ssh directory, if they exist
```
3. Check the directory listing to see if you already have a public SSH key. By default, the filenames of supported public keys for GitHub are one of the following.
- id_rsa.pub
- id_ecdsa.pub
- id_ed25519.pub

:::tipTip: If you receive an error that ~/.ssh doesn't exist, you do not have an existing SSH key pair in the default location. You can create a new SSH key pair in the next step.
:::

4. Either generate a new SSH key or upload an existing key.

- If you don't have a supported public and private key pair, or don't wish to use any that are available, generate a new SSH key.

- If you see an existing public and private key pair listed (for example, id_rsa.pub and id_rsa) that you would like to use to connect to GitHub, you can add the key to the ssh-agent.

For more information about generation of a new SSH key or addition of an existing key to the ssh-agent, see ["Generating a new SSH key and adding it to the ssh-agent."](./ssh_generate_key.md).
</TabItem>
</Tabs>


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';