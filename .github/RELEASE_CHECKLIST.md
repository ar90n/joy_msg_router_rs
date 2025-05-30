# Release Checklist

This checklist should be followed when creating a new release.

## Pre-release

- [ ] All tests pass locally (`cargo test`)
- [ ] Code passes clippy (`cargo clippy -- -D warnings`)
- [ ] Code is formatted (`cargo fmt`)
- [ ] Documentation is up to date
- [ ] Version number is updated in `Cargo.toml`
- [ ] Version number is updated in `package.xml`

## Release Process

1. **Merge Pull Requests**: Ensure all PRs for the release are merged to main

2. **Check Draft Release**: 
   - Go to the [Releases page](https://github.com/your-username/joy_msg_router_rs/releases)
   - Review the draft release created by release-drafter
   - Edit the release notes if needed
   - Verify the version number is correct

3. **Publish Release**:
   - Click "Publish release" on the draft
   - This will trigger the release workflow to build and upload artifacts

4. **Automated Build**: The release workflow will automatically:
   - Build debian packages for multiple ROS distributions
   - Build standalone binaries for x86_64 and aarch64
   - Upload all artifacts to the release

## Post-release

- [ ] Verify all artifacts are uploaded to the release
- [ ] Test installation from release artifacts
- [ ] Update any dependent repositories
- [ ] Announce release (if applicable)

## Manual Version Bump

If you need to manually set the version before release:

```bash
# Update version in both files
./scripts/bump_version.sh 0.2.0

# Commit the changes
git add Cargo.toml package.xml
git commit -m "chore: bump version to 0.2.0"
git push origin main
```