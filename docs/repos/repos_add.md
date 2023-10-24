# Add Repository

Too add a new project repository please follow these steps:
        
1.  Make sure you are at your repository folder

        cd ~/<your repository>

2.  Install CITROS to your project


        pip install citros

3.  Login to your CITROS account


        citros login

4.  Sync your repository to CITROS 


        citros init


5.  Uploud the project to the web 


        citros docker-build-push


If by any chance you did `citros init` befor you logged in and you want to add the project to CITROS then you need to run the command: 
```
citros add-remote
```
If you encounter any issues with authentication, please refer to [this](/docs/authentication/ssh/ssh_overview.md) resource for assistance.

