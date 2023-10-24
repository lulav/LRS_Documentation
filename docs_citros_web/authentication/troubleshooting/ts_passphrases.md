# Recovering your SSH key passphrase

If you've lost your SSH key passphrase, depending on the operating system you use, you may either recover it or you may need to generate a new SSH key passphrase.

<Tabs groupId="operating-systems">

<TabItem value="Windows" label="Windows">

If you lose your SSH key passphrase, there's no way to recover it. You'll need to [generate a brand new SSH keypair](/docs_citros_web/authentication/ssh/ssh_generate_key.md) 

</TabItem>
  

<TabItem value="Mac" label="MacOS">

If you configured your [SSH passphrase with the macOS keychain](/docs_citros_web/authentication/ssh/ssh_passphrases.md), you may be able to recover it.

1. In Finder, search for the Keychain Access app.

2. In Keychain Access, search for SSH.

3. Double click on the entry for your SSH key to open a new dialog box.

4. In the lower-left corner, select Show password.

5. You'll be prompted for your administrative password. Type it into the "Keychain Access" dialog box.

6. Your password will be revealed.

</TabItem>
  

<TabItem value="Linux" label="Linux">

If you lose your SSH key passphrase, there's no way to recover it. You'll need to [generate a brand new SSH keypair](/docs_citros_web/authentication/ssh/ssh_generate_key.md).

</TabItem>
</Tabs>



import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';