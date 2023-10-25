# Add Repository

## Prequisites

To add repository to CITROS you first need to Install CITROS
   
   ```
   pip install citros
   ```

## Add Repository to CITROS From Scrach

1. Go to your repository path

    ```
    cd <repository name>
    ```

2.  Login to your CITROS account

    ```
    citros login
    ```

3.  Sync your repository to CITROS 

    ```
    citros init
    ```

You should now see your repository at your [CITROS](https://citros.io/) account

## Add Repository to CITROS After Working Offline

If you started working with CITROS without login, meaning you already ini

1. Go to your repository path

    ```
    cd <repository name>
    ```

2. Sync your repository to CITROS 

    ```
    citros init
    ```

3. Login to your CITROS account

    ```
    citros login
    ```

4. Add your repository into CITROS account

    ```
    citros add-remote
    ```

You should now see your repository at your [CITROS](https://citros.io/) account

:::noteNote:
Only Metadata files are uploaded into CITROS servers, never your code
:::
