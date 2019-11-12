FROM nvcr.io/nvidian/isaac-sdk/isaac-sdk
RUN apt-get update
RUN apt-get install ssh jq -y
RUN groupadd -r isaac-ci
RUN useradd -r -g isaac-ci -d /home/isaac-ci isaac-ci
USER isaac-ci
ENV USER isaac-ci
ENV LOGNAME isaac-ci