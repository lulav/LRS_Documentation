# Generate Key

You can generate a new SSH key on your local machine. After you generate the key, you can add the public key to your account on GitHub.com to enable authentication for Git operations over SSH.

<Tabs groupId="operating-systems">
  
<TabItem value="Windows" label="Windows">

- Open Bash
- Insert the following command
```bash 
ssh-keygen -t ed25519 -C "your_email@example.com"
```
The above command creates a new SSH key, using the provided email as a label.
   
```bash 
Generating public/private ed25519 key pair.
```

- Decide on location to save the key:
- **Option 1** - Press Enter to save the default location
```bash
Enter a file in which to save the key (/c/Users/YOU/.ssh/id_ALGORITHM):
```
- **Option 2** - If you already generated and saved SSH key in the same location you will be asked to overwrite it, in this case we recommend creating a custom-named SSH key. To do so, type the default file location and replace id_ssh_keyname with your custom key name:
```bash
/c/Users/YOU/.ssh/id_ALGORITHM_new
```
- Decide whether you would like to enter [passphrase](/docs/authentication/ssh/ssh_passphrases.md). If you don't want to add it, just press Enter.
```bash
Enter passphrase (empty for no passphrase):
```
- Re-enter passphras (or just press Enter)
```bash
Enter same passphrase again: 
```
- If key generating suucceeded you will get the following details:
1. Your identification location path.
2. Your public key location path.
3. The key fingerprint.
4. The key's randomart image.

- **To copy your public ssh key use below command**
```bash
clip < ~/.ssh/id_rsa.pub
```
**Please Note**, if you changed the name of the key, then copy if from "Your public key location path"

- To add the key into CITROS follow [Add Key to CITROS](/docs/authentication/ssh/ssh_add_new.md). 


</TabItem>
  

<TabItem value="Mac" label="MacOS">

- Open Terminal
- Insert the following command
```shell 
ssh-keygen -t ed25519 -C "your_email@example.com"
```
The above command creates a new SSH key, using the provided email as a label.
   
```shell 
Generating public/private ed25519 key pair.
```

- Decide on location to save the key:
- **Option 1** - Press Enter to save the default location
```shell
Enter file in which to save the key (/User/name/default_location/id_ssh_keyname):
```
- **Option 2** - If you already generated and saved SSH key in the same location you will be asked to overwrite it, in this case we recommend creating a custom-named SSH key. To do so, type the default file location and replace id_ssh_keyname with your custom key name:
```shell
/User/name/default_location/id_ssh_keyname_updated
```
- Decide whether you would like to enter [passphrase](/docs/authentication/ssh/ssh_passphrases.md). If you don't want to add it, just press Enter.
```shell
Enter passphrase (empty for no passphrase):
```
- Re-enter passphras (or just press Enter)
```shell
Enter same passphrase again: 
```
- If key generating suucceeded you will get the following details:
1. Your identification location path.
2. Your public key location path.
3. The key fingerprint.
4. The key's randomart image.

- To copy your public ssh key use below command
```shell
pbcopy < ~/.ssh/id_ed25519.pub
```
**Please Note**, if you changed the name of the key, then copy if from "Your public key location path"

- To add the key into CITROS follow [Add Key to CITROS](/docs/authentication/ssh/ssh_add_new.md). 

</TabItem>


<TabItem value="Linux" label="Linux">

- Open Terminal
- Insert the following command
```shell 
ssh-keygen -t ed25519 -C "your_email@example.com"
```
The above command creates a new SSH key, using the provided email as a label.
   
```shell 
Generating public/private ed25519 key pair.
```

- Decide on location to save the key:
- **Option 1** - Press Enter to save the default location
```shell
Enter file in which to save the key (/home/YOU/.ssh/ALGORITHM):
```
- **Option 2** - If you already generated and saved SSH key in the same location you will be asked to overwrite it, in this case we recommend creating a custom-named SSH key. To do so, type the default file location and replace id_ssh_keyname with your custom key name:
```shell
/home/YOU/.ssh/id_ALGORITHM_new
```
- Decide whether you would like to enter [passphrase](/docs/authentication/ssh/ssh_passphrases.md). If you don't want to add it, just press Enter.
```shell
Enter passphrase (empty for no passphrase):
```
- Re-enter passphras (or just press Enter)
```shell
Enter same passphrase again: 
```
- If key generating suucceeded you will get the following details:
1. Your identification location path.
2. Your public key location path.
3. The key fingerprint.
4. The key's randomart image.

- To copy your public ssh key use below command
```shell
cat ~/.ssh/id_rsa.pub
```
**Please Note**, if you changed the name of the key, then copy if from "Your public key location path"

- To add the key into CITROS follow [Add Key to CITROS](./ssh_add_new.md). 

</TabItem>
</Tabs>




import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';