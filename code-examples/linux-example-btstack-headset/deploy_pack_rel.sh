echo "<version>${CE_REL_VERSION}.${CI_PIPELINE_IID}</version>" > version.xml
zip -r deploy_fd/bt-linux-ce.zip * -x *.sh --exclude=*ossscanreport* --exclude=*_coverity_*
#zip -r deploy_fd/bluetooth-linux.zip CMakeLists.txt *.c *.h docs/*.* README.md RELEASE.md LICENSE.txt version.xml linux-audio images wiced_hal
