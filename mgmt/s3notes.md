# Notes On Everything S3

## AWS
aws.amazon.com/console

user id:  robin.knight@roadnarrows.com
passwd:   xxxx

## Authority

### s3:
userid: gort
passwd: forbiddenplanet

### Passphrase:
earth stood still

---

## Tools

### aptly

### s3cmd
#### Sync media
```
cd /prj
s3cmd sync media/ s3://media.roadnarrows.com
```
Note that the trailing '/' on the source directory media. Add, otherwise
media becomes a directory under the destination.

---

## Repo Staging

### rnmake_deb_all
The tool rnmake_deb_all makes all publically release SDK devel packages
```
rnmake_deb_all
```

### Make an individual deb package

```
cd /prj/pkg/<PKG>
make clobber; make deps; make install; make deb-pkg-deb
```

### Replace any old packages with new packages in /public/repostaging
$ rm <PKG>-dev-*.deb

#### On host
$ cp /prj/pkg/<PKG>/dist/dist.<RNARCH>/<PKG>-dev-<x.y.z>-<ARCH>.deb /public/repostaging/.

#### On target
$ scp /prj/pkg/<PKG>/dist/dist.<RNARCH>/<PKG>-dev-<x.y.z>-<ARCH>.deb rknight@sharetheroad:/public/repostaging/.

---

## Repo trusty main

### One time setup actions
```
aptly repo create main

aptly repo add main ../repostaging/

aptly publish repo -distribution=trusty -component=main main s3:distro.roadnarrows.com:repo-stable
```

### Repo Updates

#### 1. Replace any old packages with new packages (see above)

#### 2. Then add
```
aptly repo add main ../repostaging/
```

#### 3. Finally, update reposistory
```
aptly publish update trusty  s3:distro.roadnarrows.com:repo-stable
```

---

## Repo trusty unstable

### One time setup actions
```
aptly repo create unstable

aptly repo add unstable ../repostaging/

aptly publish repo -distribution=trusty -component=unstable unstable s3:distro.roadnarrows.com:repo
```

### Repo Updates

#### 1. Replace any old packages with new packages (see above)

#### 2. Then add
```
aptly repo add unstable ../repostaging/
```

#### 3. Finally, update reposistory
```
aptly publish update trusty  s3:distro.roadnarrows.com:repo
```
