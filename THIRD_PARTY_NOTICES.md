# Third-Party Notices

This file records third-party material redistributed in the source
repository. Runtime dependencies installed through the operating system or
rosdep are not copied into this repository and are governed by their own
packages.

## ROS package manifest schemas

Files:

- `tools/schema/package_format3.xsd`
- `tools/schema/package_common.xsd`

Provenance:

- upstream repository: `https://github.com/ros-infrastructure/rep`
- upstream commit: `11ca24a41f31480dfb9562ba99f2a5b93d3ebda5`
- upstream paths: `xsd/package_format3.xsd` and `xsd/package_common.xsd`
- associated specification: REP-149, Package Manifest Format Three

Licensing basis:

REP-149 explicitly places the specification in the public domain and links
`package_format3.xsd` as its schema. The pinned upstream repository snapshot
does not contain a separate repository-level license file, and the XSD files
do not carry their own license header. This provenance and limitation are
recorded rather than assigning a new license to the upstream files.

The copies are used unchanged so package metadata validation can run without a
network request. When updating them, update both files together, pin the new
upstream commit, compare the exact diff, and re-check the licensing basis.
