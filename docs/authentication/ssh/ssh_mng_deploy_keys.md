# Managing Deploy Keys

You can launch projects from a repository on [CITROS.io](https://citros.io/) to your server by using a deploy key, which is an SSH key that grants access to a single repository. CITROS attaches the public part of the key directly to your repository instead of a personal account, and the private part of the key remains on your server.

## Set Up Deploy Keys

1. [Generate SSH-Key](/docs/authentication/ssh/ssh_generate_key.md) on your server, and remember where you save the generated public and private rsa key pair.

2. [Navigate to Settings](https://citros.io/settings).

3. Open [SSH Keys Tab](https://citros.io/settings?tab=ssh_keys).

4. Click "New SSH Key" button

5. Enter SSH key name.

6. Paste the public key.

7. Click "Add" button to add the SSH Key to the account.

8. The new key will be added to the list item.