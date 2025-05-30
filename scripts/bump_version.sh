#!/bin/bash
# Script to bump version in both Cargo.toml and package.xml

set -e

if [ $# -ne 1 ]; then
    echo "Usage: $0 <new_version>"
    echo "Example: $0 0.2.0"
    exit 1
fi

NEW_VERSION=$1

# Update Cargo.toml
echo "Updating Cargo.toml..."
sed -i "s/^version = \".*\"/version = \"$NEW_VERSION\"/" Cargo.toml

# Update package.xml
echo "Updating package.xml..."
sed -i "s/<version>.*<\/version>/<version>$NEW_VERSION<\/version>/" package.xml

echo "Version bumped to $NEW_VERSION"
echo ""
echo "Files modified:"
echo "  - Cargo.toml"
echo "  - package.xml"
echo ""
echo "Please review the changes and commit them:"
echo "  git add Cargo.toml package.xml"
echo "  git commit -m \"chore: bump version to $NEW_VERSION\""
echo ""
echo "To create a release:"
echo "  git tag -a v$NEW_VERSION -m \"Release v$NEW_VERSION\""
echo "  git push origin main"
echo "  git push origin v$NEW_VERSION"