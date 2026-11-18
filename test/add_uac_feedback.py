# Build components/usb_host_uac/uac_feedback.c into the native test binary.
Import("env")
env.BuildSources("$BUILD_DIR/uac_feedback", "$PROJECT_DIR/components/usb_host_uac", src_filter="+<uac_feedback.c>")
