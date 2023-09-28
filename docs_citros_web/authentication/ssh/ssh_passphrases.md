# SSH key Passphrases

You can secure your SSH keys and configure an authentication agent so that you won't have to reenter your passphrase every time you use your SSH keys.

## About passphrases for SSH keys
With SSH keys, if someone gains access to your computer, the attacker can gain access to every system that uses that key. To add an extra layer of security, you can add a passphrase to your SSH key. To avoid entering the passphrase every time you connect, you can securely save your passphrase in the SSH agent.

## Adding or changing a passphrase
You can change the passphrase for an existing private key without regenerating the keypair by typing the following command:
```shell
ssh-keygen -p -f ~/.ssh/id_ed25519
```
- If your key already has a passphrase, you will be prompted to enter it before you can change to a new passphrase.
```shell
Enter old passphrase:
```
```shell
Key has comment 'your_email@example.com'
```
- Enter new passphrase
```shell
Enter new passphrase (empty for no passphrase):
```
- Re-Enter new passphrase
```shell
Enter same passphrase again: [Repeat the new passphrase]
```
- Your ssh should have a new passphrase
```shell
Your identification has been saved with the new passphrase.
```

