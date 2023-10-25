# Settings Overview

Settings screen contains a few tabs which represent eaach option a user has.
Some of the settings are open only to admin users.

## Table of Contents
1. [Access to Settings Screen](#access-to-settings-screen)
2. [General Screen](#general-screen)
3. [Organization](#organization-screen)
4. [Users](#users-screen)
5. [Change Password](#change-password-screen)
6. [SSH Keys](#ssh-keys)

## Access to Settings Screen
The settings screen can be entered in two ways

1. from the gear in the top right corner of the screen. this option is avaialable throughout all screen.

    1. Click on Gear icon
    2. Choose Settings
![image](img/setting_intro1.png)

2. From settings tab which available in your home page

![image](img/setting_intro2.png)

## General Screen

General screen contains general details

- First Name
- Last Name 
- Email address

The first and last name are changable
1. Click on the box to edit
2. After editing is done, Click "Save Changes" button

![image](img/general.png)

The email address cannot be changed.

## Organization Screen

Organization screen contains organization details 

- Hosting Type
- Organization Type 
- Name

This details cannot be changed.

![image](img/organiztion.png)

## Users Screen

Users Screen contains a list of all users in the organization.

Each list item contains
- First Name
- Last Name
- Email
- User Type (user or admin)
- User Activation Toggle
- User Status Active/ Inactive
- Date added to organiztion

![image](img/users_list.png)

### Options

#### Search Box

This allows to search a user by enetering an email in the text box.

#### Admin Only Options

##### Invite Users

1. Click on "Invite User" button

![image](img/users_invite.png)

2. Follow the guide to [Invite a New User](/docs/authentication/account/account_login#invite-a-new-user-to-your-organization)

##### User Activation

Toggle to activate or deactivate the user in the user list.

## Change Password Screen

The change password screen offers you to update your password.
You must have your old password to update your password.

To change password:
1. Insert old password

2. Insert new password. Password must be at least 8 characters 

3. Repeat new password.

4. Click "Save Changes" button.

![image](img/pass.png)

## SSH Keys

SSH Keys screen contains a list of all SSH Keys added to your CITROS account

### Options

#### Add SSH Key to List

1. [Generate SSH Key](/docs/authentication/ssh/ssh_generate_key.md)

2. [Add New Key](/docs/authentication/ssh/ssh_add_new.md).

3. Click "New SSH Key" button

4. Enter SSH key name.

5. Paste the SSH key copied to clipbord.

6. Click "Add" button to add the SSH Key to the account.

7. The new key will be added to the list item

#### Delete Key 

1. Click the Delete Button to the right of the key.

2. Confirm deletion or cancel otherwise

![Alt text](img/key_delete.png)



