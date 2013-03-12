/usr/share/kernel-package/ruleset/kernel_version.mk.orig 2008-05-02 07:06:28.000000000 +0200
+++ /usr/share/kernel-package/ruleset/kernel_version.mk 2010-07-08 00:02:45.316669641 +0200
@ -62,7 +62,7 @@
@echo "$(strip $(EXTRAVERSION))"

debian_LOCALVERSION:
- @echo $(if $(strip $(localver-full)),"$(strip $(localver-full))", "$(strip $(LOCALVERSION))")
+ @./scripts/setlocalversion

debian_TOPDIR:
# 2.6 kernels declared TOPDIR obsolete, so use srctree if it exists
