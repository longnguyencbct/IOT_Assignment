FROM postgres:17
ENV POSTGRES_USER=admin
ENV POSTGRES_PASSWORD=admin
ENV POSTGRES_DB=iot_database

# Copy the initialization script
COPY init_query.sql /docker-entrypoint-initdb.d/