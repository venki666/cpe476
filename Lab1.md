## Installing Python and support packages

### Check and Install the latest Python Interperter

By default Ubuntu 18.04 or Ubuntu 20.04 come with Python pre-installed. [Check your version of Python](https://phoenixnap.com/kb/check-python-version) by entering the following:

```output
python ––version
```

If the revision level is lower than 3.8, or if Python is not installed, continue to the next step, else jump to [virtualenv](#viren)

Open a terminal window, and enter the following:

```output
sudo apt update
```

Install Supporting Software

The software-properties-common package gives you better control over your package manager by letting you add PPA (Personal Package Archive) repositories. Install the supporting software with the command:

```output
sudo apt install software-properties-common
```

Add the deadsnakes PPA to your system’s sources list:

```
sudo add-apt-repository ppa:deadsnakes/ppa
```

When prompted press `Enter` to continue:

Once the repository is enabled, install Python 3.8 with:

```
sudo apt install python3.8
```

Verify that the installation was successful by typing:

```
python3.8 --version
```

<a name="viren"></a>

### Setting Up a Virtual Environment

Virtual environments enable you to have an isolated space on your server for Python projects, ensuring that each of your projects can have its own set of dependencies that won’t disrupt any of your other projects.

Setting up a programming environment provides greater control over Python projects and over how different versions of packages are handled. This is especially important when working with third-party packages.

You can set up as many Python programming environments as you would like. Each environment is basically a directory or folder on your server that has a few scripts in it to make it act as an environment.

While there are a few ways to achieve a programming environment in Python, we’ll be using the **venv** module here, which is part of the standard Python 3 library. Let’s install venv by typing:

```
sudo apt install -y python3-venv
```

With this installed, we are ready to create environments. Let’s either choose which directory we would like to put our Python programming environments in, or create a new directory with `mkdir`, as in:

```
mkdir environments
cd environments
```

Once you are in the directory where you would like the environments to live, you can create an environment by running the following command:

```
python3 -m venv my_env
```

Essentially, `pyvenv` sets up a new directory that contains a few items which we can view with the `ls` command:

```
ls my_env
Outputbin include lib lib64 pyvenv.cfg share
```

Together, these files work to make sure that your projects are isolated from the broader context of your server, so that system files and project files don’t mix. This is good practice for version control and to ensure that each of your projects has access to the particular packages that it needs. Python Wheels, a built-package format for Python that can speed up your software production by reducing the number of times you need to compile, will be in the Ubuntu 20.04 `share` directory.

To use this environment, you need to activate it, which you can achieve by typing the following command that calls the **activate** script:

```
source my_env/bin/activate
```

Your command prompt will now be prefixed with the name of your environment, in this case it is called my_env. Depending on what version of Debian Linux you are running, your prefix may appear somewhat differently, but the name of your environment in parentheses should be the first thing you see on your line.

This prefix lets us know that the environment my_env is currently active, meaning that when we create programs here they will use only this particular environment’s settings and packages.

**Note:** Within the virtual environment, you can use the command `python` instead of `python3`, and `pip` instead of `pip3` if you would prefer. If you use Python 3 on your machine outside of an environment, you will need to use the `python3` and `pip3` commands exclusively.

After following these steps, your virtual environment is ready to use.

### Creating a “Hello, World” Program

Now that we have our virtual environment set up, let’s create a traditional “Hello, World!” program. This will let us test our environment and provides us with the opportunity to become more familiar with Python if we aren’t already.

To do this, we’ll open up a command-line text editor such as nano and create a new file:

```
nano hello.py
```

Once the text file opens up in the terminal window we’ll type out our program:

```python
print("Hello, World!")
```

Copy

Exit nano by typing the `CTRL` and `X` keys, and when prompted to save the file press `y`.

Once you exit out of nano and return to your shell, let’s run the program:

```
python hello.py
```

The `hello.py` program that you just created should cause your terminal to produce the following output:

```
OutputHello, World!
```

To leave the environment, type the command `deactivate` and you will return to your original directory.