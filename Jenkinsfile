@Library('xmos_jenkins_shared_library@v0.34.0') _

getApproval()

def runningOn(machine) {
    println "Stage running on:"
    println machine
}

def runForEach(folders, Closure body) {
  folders.each { app -> body(app) }
}

def buildDocs() {
    withVenv {
        sh 'pip install git+ssh://git@github.com/xmos/xmosdoc@${XMOSDOC_VERSION}'
        sh 'xmosdoc'
        zip zipFile: "${REPO}_docs.zip", archive: true, dir: 'doc/_build'
    }
}

pipeline {
  agent none

  options {
    timestamps()
    buildDiscarder(xmosDiscardBuildSettings(onlyArtifacts=false))
  }

  environment {
    REPO = 'lib_device_control'
    XMOSDOC_VERSION = 'v6.1.1'
  } // environment

  parameters {
    string(
      name: 'TOOLS_VERSION',
        defaultValue: '15.3.0',
        description: 'The XTC tools version'
    )
  }
  stages {
    stage('Cross-platform builds and tests') {
      parallel {
        stage('Library checks, tests and Linux x86_64 host builds') {
          agent {
            label 'linux&&64'
          }

          stages {
            stage("Clone library")
            {
              steps {
                runningOn(env.NODE_NAME)
                dir("${REPO}") {
                  // clone the repo and checkout the changes
                  checkout scm
                }
              }
            }
            stage('xCORE builds') {
              steps {
                // build all the supported firmware applications
                runForEach(['i2c', 'i2c/host_xcore', 'spi', 'usb', 'xscope']) { app ->
                  withTools(params.TOOLS_VERSION) { // the XTC tools are necessary to build the XSCOPE host application
                    dir("${REPO}/examples/${app}") {
                          sh 'cmake -G "Unix Makefiles" -B build'
                          sh "xmake -C build"
                    }
                  }
                }
              }
            }
            stage('Library checks') {
              steps {
                runLibraryChecks("${WORKSPACE}/${REPO}", "v2.0.1")
              }
            }
            stage('Tests') {
              steps {
                dir("${REPO}") {
                  createVenv(reqFile: "requirements.txt")
                  withVenv {
                    withTools(params.TOOLS_VERSION) {
                      dir("tests") {
                        runPytest("--dist worksteal")
                      }
                    }
                  }
                }
              }
            }
            stage('Linux x86_64 host builds') {
              steps {
                // build all the supported host applications
                runForEach(['usb', 'xscope']) { app ->
                  withTools(params.TOOLS_VERSION) { // the XTC tools are necessary to build the XSCOPE host application
                    dir("${REPO}/examples/${app}/host") {
                      sh "cmake -B build"
                      sh "make -C build"
                    }
                  }
                }
              }
            }
          }
          post {
            cleanup {
              xcoreCleanSandbox()
            }
          }
        }

        stage('Build documentation') {
          agent {
            label 'documentation'
          }
          stages {
            stage('Docs') {
              steps {
<<<<<<< HEAD
                createVenv(reqFile: "requirements.txt")
                withVenv {
                  sh "pip install git+ssh://git@github.com/xmos/xmosdoc@${XMOSDOC_VERSION}"
                  sh 'xmosdoc'
                }
                // Zip and archive doc files
                zip dir: "doc/_build/html/", zipFile: "lib_device_control_docs_html.zip"
                archiveArtifacts artifacts: "lib_device_control_docs_html.zip"
                zip dir: "doc/_build/pdf/", zipFile: "lib_device_control_docs_pdf.zip"
                archiveArtifacts artifacts: "lib_device_control_docs_pdf.zip"
=======
                runningOn(env.NODE_NAME)
                createVenv("requirements.txt")
                buildDocs()
>>>>>>> fde69046e9cc1913f02bf342508605e62415b7d5
              }
            }
          }
          post {
            cleanup {
              xcoreCleanSandbox()
            }
          }
        }

        stage('RPI host builds') {
          agent {
            label 'armv7l&&raspian'
          }
          stages {
            stage('Build') {
              steps {
                runningOn(env.NODE_NAME)
                // clone the repo and checkout the changes
                checkout scm
                // build all the supported host applications
                runForEach(['i2c/host_rpi', 'spi/host']) { app ->
                  dir("examples/${app}") {
                      sh "cmake -B build"
                      sh "make -C build"
                  }
                }
              }
            }
          }
          post {
            cleanup {
              xcoreCleanSandbox()
            }
          }
        } // RPI host builds

        stage('Mac x86_64 host builds') {
          agent {
            label 'macOS&&x86_64'
          }
          stages {
            stage('Build') {
              steps {
                runningOn(env.NODE_NAME)
                // clone the repo and checkout the changes
                checkout scm
                // build all the supported host applications
                runForEach(['usb', 'xscope']) { app ->
                  withTools(params.TOOLS_VERSION) { // the XTC tools are necessary to build the XSCOPE host application
                    dir("examples/${app}/host") {
                      sh "cmake -B build"
                      sh "make -C build"
                    }
                  }
                }
              }
            }
          }
          post {
            cleanup {
              xcoreCleanSandbox()
            }
          }
        } // Linux x86_64 host suilds

        stage('Mac arm64 host builds') {
          agent {
            label 'macos&&arm64'
          }
          stages {
            stage('Build') {
              steps {
                runningOn(env.NODE_NAME)
                // clone the repo and checkout the changes
                checkout scm
                // build all the supported host applications
                runForEach(['usb', 'xscope']) { app ->
                  withTools(params.TOOLS_VERSION) { // the XTC tools are necessary to build the XSCOPE host application
                    dir("examples/${app}/host") {
                      sh "cmake -B build"
                      sh "make -C build"
                    }
                  }
                }
              }
            }
          }
          post {
            cleanup {
              xcoreCleanSandbox()
            }
          }
        } // Mac arm64 host builds

        stage('Win32 host builds') {
          agent {
            label 'sw-bld-win0'
          }
          stages {
             stage('Build') {
              steps {
                runningOn(env.NODE_NAME)
                // clone the repo and checkout the changes
                checkout scm

                // Build the USB host example for 32 bit as libusb is 32 bit
                withVS('vcvars32.bat') {
                  dir("examples/usb/host") {
                    sh "cmake -G Ninja -B build"
                    sh "ninja -C build"
                  }
                }
                // Build the XCOPE host example for 64 bit as XTC tools  32 bit
                withVS('vcvars64.bat') {
                  withTools(params.TOOLS_VERSION) {
                    dir("examples/xscope/host") {
                      sh "cmake -G Ninja -B build"
                      sh "ninja -C build"
                    }
                  }
                }
              }
            }
          }
          post {
            cleanup {
              xcoreCleanSandbox()
            }
          }
        } // Win32 host builds

      } // parallel
    } // Cross-platform Builds & Tests
  } // stages
}
