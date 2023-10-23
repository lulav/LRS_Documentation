# Managing deploy keys

You can launch projects from a repository on GitHub.com to your server by using a deploy key, which is an SSH key that grants access to a single repository. GitHub attaches the public part of the key directly to your repository instead of a personal account, and the private part of the key remains on your server.

## Set up deploy keys

1. [Run the ssh-keygen procedure](/docs_citros_web/authentication/ssh/ssh_generate_key.md) on your server, and remember where you save the generated public and private rsa key pair.

2. On citros.com, navigate to the main page of the repository.

3. Under your repository name, click  Settings. If you cannot see the "Settings" tab, select the  dropdown menu, then click Settings.

4. In the sidebar, click Deploy Keys.

5. Click Add deploy key.

6. In the "Title" field, provide a title.

7. In the "Key" field, paste your public key.

8. Select Allow write access if you want this key to have write access to the repository. A deploy key with write access lets a deployment push to the repository.

9. Click Add key.