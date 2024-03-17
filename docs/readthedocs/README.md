# DART Documentation

## Build Documentation Locally

To install dependencies to build the documentation, navigate to `<dart_root>/docs/readthedocs` and run:

```console
pip install -r .\requirements.txt
```

To build the documentation using Sphinx, navigate to `<dart_root>/docs/readthedocs` and run:

```console
./make.bat html # on Windows
make html # on other platforms
```

Note that on Windows, you need to use make.bat instead of make. Additionally, you may need to remove the _build directory before building to ensure a clean build.

## Build Multi Language Documentation

1. To generate .pot files, navigate to <dart_root>/docs/readthedocs and run the following command:

   ```console
   sphinx-build -b gettext . _build/gettext
   ```

1. Create a directory with a translated file for each target language. For example, to create Korean documentation, run the following command:

   ```console
   sphinx-intl update -p _build/gettext -l ko
   ```

   This command will create a directory structure similar to the following (with one .po file per .rst file in your documentation):

   ```console
   locales
   └── ko
       └── LC_MESSAGES
           └── index.po
   ```

   You can now open those .po files with a text editor and translate them, taking care not to break the reStructuredText notation. For example:

   ```rst
   # b8f891b8443f4a45994c9c0a6bec14c3
   #: ../../index.rst:4
   msgid ""
   "Read the Docs hosts documentation for the open source community."
   "It supports :ref:`Sphinx <sphinx>` docs written with reStructuredText."
   msgstr ""
   "FILL HERE BY TARGET LANGUAGE FILL HERE BY TARGET LANGUAGE FILL HERE "
   "BY TARGET LANGUAGE :ref:`Sphinx <sphinx>` FILL HERE."
   ```

1. (Optional) You can create a pull request to incorporate the changes into dart.readthedocs.io. Alternatively, you can build the documentation locally in the target language by running the following command:

   ```console
   sphinx-build -b html -D language=ko . _build/html/ko
   ```
