# Add Repository
Too add a new project repository you need to follow these steps:

**Be sure first that you are in your repository folder**

1. Install CITROS to your project

```
pip install citros
```

2. Login to your CITROS account
```
citros login
```

3. Sync your repo to CITROS 
```
citros init
```

4. Uploud the project to the web 
```
citros docker-build-push
```

If by any chance you did `citros init` befor you logged in and you want to add the project to CITROS then you need to run the command: 
```
citros add-remote
```
**Be sure to run the `setup-ssh` command once before running the `add-remote` command.**
