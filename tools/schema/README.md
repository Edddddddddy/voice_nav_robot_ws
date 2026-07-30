# Vendored ROS package schema

`package_format3.xsd` and `package_common.xsd` are the schemas referenced by
REP-149. They are vendored so `ament_xmllint` does not depend on the availability
or completeness of an HTTP response during local or CI tests.

Source:

- repository: `https://github.com/ros-infrastructure/rep`
- commit: `11ca24a41f31480dfb9562ba99f2a5b93d3ebda5`
- paths: `xsd/package_format3.xsd`, `xsd/package_common.xsd`
- licensing basis: REP-149 places the specification in the public domain and
  links `package_format3.xsd` as its schema; the pinned repository snapshot
  does not contain a separate repository-level license file

`catalog.xml` maps the canonical ROS schema URL used by `package.xml` files to
the local copy. Update both XSD files together and record the new source commit.
See the root `THIRD_PARTY_NOTICES.md` for redistribution provenance.
