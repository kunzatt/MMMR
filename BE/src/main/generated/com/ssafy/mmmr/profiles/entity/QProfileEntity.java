package com.ssafy.mmmr.profiles.entity;

import static com.querydsl.core.types.PathMetadataFactory.*;

import com.querydsl.core.types.dsl.*;

import com.querydsl.core.types.PathMetadata;
import javax.annotation.processing.Generated;
import com.querydsl.core.types.Path;
import com.querydsl.core.types.dsl.PathInits;


/**
 * QProfileEntity is a Querydsl query type for ProfileEntity
 */
@Generated("com.querydsl.codegen.DefaultEntitySerializer")
public class QProfileEntity extends EntityPathBase<ProfileEntity> {

    private static final long serialVersionUID = 929677827L;

    private static final PathInits INITS = PathInits.DIRECT2;

    public static final QProfileEntity profileEntity = new QProfileEntity("profileEntity");

    public final com.ssafy.mmmr.account.entity.QAccountEntity account;

    public final EnumPath<CallSign> callSign = createEnum("callSign", CallSign.class);

    public final DateTimePath<java.time.LocalDateTime> createdAt = createDateTime("createdAt", java.time.LocalDateTime.class);

    public final BooleanPath deleted = createBoolean("deleted");

    public final NumberPath<Long> id = createNumber("id", Long.class);

    public final StringPath nickname = createString("nickname");

    public final DateTimePath<java.time.LocalDateTime> updatedAt = createDateTime("updatedAt", java.time.LocalDateTime.class);

    public QProfileEntity(String variable) {
        this(ProfileEntity.class, forVariable(variable), INITS);
    }

    public QProfileEntity(Path<? extends ProfileEntity> path) {
        this(path.getType(), path.getMetadata(), PathInits.getFor(path.getMetadata(), INITS));
    }

    public QProfileEntity(PathMetadata metadata) {
        this(metadata, PathInits.getFor(metadata, INITS));
    }

    public QProfileEntity(PathMetadata metadata, PathInits inits) {
        this(ProfileEntity.class, metadata, inits);
    }

    public QProfileEntity(Class<? extends ProfileEntity> type, PathMetadata metadata, PathInits inits) {
        super(type, metadata, inits);
        this.account = inits.isInitialized("account") ? new com.ssafy.mmmr.account.entity.QAccountEntity(forProperty("account")) : null;
    }

}

