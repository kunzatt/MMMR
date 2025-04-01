package com.ssafy.mmmr.devices.entity;

import static com.querydsl.core.types.PathMetadataFactory.*;

import com.querydsl.core.types.dsl.*;

import com.querydsl.core.types.PathMetadata;
import javax.annotation.processing.Generated;
import com.querydsl.core.types.Path;
import com.querydsl.core.types.dsl.PathInits;


/**
 * QHomeDeviceEntity is a Querydsl query type for HomeDeviceEntity
 */
@Generated("com.querydsl.codegen.DefaultEntitySerializer")
public class QHomeDeviceEntity extends EntityPathBase<HomeDeviceEntity> {

    private static final long serialVersionUID = 2127394832L;

    private static final PathInits INITS = PathInits.DIRECT2;

    public static final QHomeDeviceEntity homeDeviceEntity = new QHomeDeviceEntity("homeDeviceEntity");

    public final com.ssafy.mmmr.account.entity.QAccountEntity account;

    public final StringPath device = createString("device");

    public final NumberPath<Long> id = createNumber("id", Long.class);

    public final StringPath turned = createString("turned");

    public QHomeDeviceEntity(String variable) {
        this(HomeDeviceEntity.class, forVariable(variable), INITS);
    }

    public QHomeDeviceEntity(Path<? extends HomeDeviceEntity> path) {
        this(path.getType(), path.getMetadata(), PathInits.getFor(path.getMetadata(), INITS));
    }

    public QHomeDeviceEntity(PathMetadata metadata) {
        this(metadata, PathInits.getFor(metadata, INITS));
    }

    public QHomeDeviceEntity(PathMetadata metadata, PathInits inits) {
        this(HomeDeviceEntity.class, metadata, inits);
    }

    public QHomeDeviceEntity(Class<? extends HomeDeviceEntity> type, PathMetadata metadata, PathInits inits) {
        super(type, metadata, inits);
        this.account = inits.isInitialized("account") ? new com.ssafy.mmmr.account.entity.QAccountEntity(forProperty("account")) : null;
    }

}

